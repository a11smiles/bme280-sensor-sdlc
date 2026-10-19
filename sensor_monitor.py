#!/usr/bin/env python3
"""
sensor_monitor.py - BME280 environmental monitor with binary telemetry framing
and automatic defect reporting to GitHub Issues.

What it does
------------
1. Samples a Bosch BME280 (temperature / humidity / pressure) over I2C.
2. Packs each sample into a compact binary telemetry frame (the format a
   constrained radio link such as LoRa or BLE would carry) and "transmits" it.
3. If ANY stage throws (sensor read, frame encoding, transmit), the exception is
   captured with full diagnostic context and filed as a GitHub issue - then the
   monitor keeps running. A fault must never take the device offline.

Issue reporting runs on a background thread, so a slow or unavailable network
never stalls sampling. Repeat occurrences of the same fault are de-duplicated
into one issue (with periodic recurrence comments). If GitHub can't be reached,
incidents are spooled to disk and retried.

Usage
-----
    export GH_ISSUE_TOKEN=github_pat_xxx      # fine-grained PAT, Issues: R/W
    python3 sensor_monitor.py                  # real sensor
    python3 sensor_monitor.py --simulate       # no hardware (rehearsal)
    python3 sensor_monitor.py --dry-run        # write issues to disk, not GitHub

Demo fault injection (backup if the room is too dry to trigger by breath):
    kill -USR1 <pid>        # next sample reports --inject-humidity (default 72.5 %RH)
"""

from __future__ import annotations

import argparse
import datetime as dt
import getpass
import hashlib
import json
import linecache
import logging
import logging.handlers
import os
import platform
import queue
import random
import shutil
import signal
import socket
import struct
import subprocess
import sys
import threading
import time
import traceback
from collections import deque
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Optional
from urllib.parse import quote

# =============================================================================
# Configuration
# =============================================================================

GITHUB_REPO = "OWNER/REPO"              # <-- set to the repo that holds this file
GITHUB_API = "https://api.github.com"
GITHUB_TOKEN_ENV = "GH_ISSUE_TOKEN"     # env var holding the PAT

TRIGGER_LABEL = "sensor-fault"          # the workflow listens for this label
ISSUE_LABELS = [TRIGGER_LABEL, "bug"]
LABEL_DEFINITIONS = {
    "sensor-fault": ("b60205", "Fault auto-reported by a field device"),
    "bug": ("d73a4a", "Something isn't working"),
}

I2C_PORT = 1
I2C_ADDRESS = 0x77                      # 0x76 on some breakout boards
BME280_CHIP_ID_REG = 0xD0
BME280_EXPECTED_CHIP_ID = 0x60          # 0x58 would indicate a BMP280 (no humidity)

SAMPLE_INTERVAL_S = 2.0
HISTORY_LEN = 30                        # samples kept for the incident report
RECURRENCE_COMMENT_COOLDOWN_S = 120     # min seconds between "still happening" comments
SPOOL_RETRY_INTERVAL_S = 60
MAX_ISSUE_BODY_CHARS = 60_000           # GitHub hard limit is 65,536

APP_NAME = "sensor-monitor"
APP_VERSION = "1.4.2"
STATE_DIR = Path(os.environ.get("SENSOR_MONITOR_HOME", Path.home() / ".sensor-monitor"))
SPOOL_DIR = STATE_DIR / "spool"
LOG_FILE = STATE_DIR / "sensor-monitor.log"

# Bosch BME280 datasheet operating ranges. Every value inside these ranges is a
# legitimate reading that the firmware must be able to handle end to end.
SENSOR_RATED_RANGES = {
    "temperature_c": (-40.0, 85.0),
    "humidity_pct": (0.0, 100.0),
    "pressure_hpa": (300.0, 1100.0),
}

# =============================================================================
# Telemetry frame (wire format)
# =============================================================================
#
# Little-endian, fixed size. Sent over a bandwidth-constrained link, so values
# are transmitted as scaled integers rather than floats.
#
#   B  magic           0xB2
#   B  version         frame format version
#   I  sequence        monotonically increasing sample counter
#   h  temperature     centi-degC     (23.47 C    -> 2347)
#   H  humidity        milli-%RH      (45.123 %   -> 45123)
#   H  pressure        deci-hPa       (1013.2 hPa -> 10132)
#   H  crc16           CRC-16/CCITT-FALSE over all preceding bytes

FRAME_MAGIC = 0xB2
FRAME_VERSION = 1
FRAME_FIELDS = ("magic", "version", "sequence", "temperature_c", "humidity_pct", "pressure_hpa")
FRAME_STRUCT = struct.Struct("<BBIhHH")
CRC_STRUCT = struct.Struct("<H")

# Fixed-point scale factor for each measurement on the wire.
FIELD_SCALES = {
    "temperature_c": 100,    # centi-degC
    "humidity_pct": 1000,    # milli-%RH: humidity is 0-100 %, so this always fits a uint16
    "pressure_hpa": 10,      # deci-hPa
}


def crc16_ccitt(data: bytes, crc: int = 0xFFFF) -> int:
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF)."""
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def encode_frame(seq: int, temperature_c: float, humidity_pct: float, pressure_hpa: float) -> bytes:
    """Pack one sample into a telemetry frame."""
    temp_scaled = round(temperature_c * FIELD_SCALES["temperature_c"])
    hum_scaled = round(humidity_pct * FIELD_SCALES["humidity_pct"])
    pres_scaled = round(pressure_hpa * FIELD_SCALES["pressure_hpa"])
    payload = FRAME_STRUCT.pack(
        FRAME_MAGIC,
        FRAME_VERSION,
        seq & 0xFFFFFFFF,
        temp_scaled,
        hum_scaled,
        pres_scaled,
    )
    return payload + CRC_STRUCT.pack(crc16_ccitt(payload))


def decode_frame(frame: bytes) -> dict:
    """Inverse of encode_frame. Raises ValueError on a malformed frame."""
    expected_len = FRAME_STRUCT.size + CRC_STRUCT.size
    if len(frame) != expected_len:
        raise ValueError(f"frame length {len(frame)} != expected {expected_len}")
    payload, crc_bytes = frame[: FRAME_STRUCT.size], frame[FRAME_STRUCT.size:]
    (crc_rx,) = CRC_STRUCT.unpack(crc_bytes)
    crc_calc = crc16_ccitt(payload)
    if crc_rx != crc_calc:
        raise ValueError(f"CRC mismatch: received 0x{crc_rx:04X}, computed 0x{crc_calc:04X}")
    magic, version, seq, t, h, p = FRAME_STRUCT.unpack(payload)
    if magic != FRAME_MAGIC:
        raise ValueError(f"bad magic 0x{magic:02X}")
    return {
        "version": version,
        "sequence": seq,
        "temperature_c": t / FIELD_SCALES["temperature_c"],
        "humidity_pct": h / FIELD_SCALES["humidity_pct"],
        "pressure_hpa": p / FIELD_SCALES["pressure_hpa"],
    }


def transmit_frame(frame: bytes) -> None:
    """Stand-in for the radio driver: log the frame that would go on air."""
    logging.getLogger(APP_NAME).debug("TX %d bytes: %s", len(frame), frame.hex(" "))


# =============================================================================
# Sensors
# =============================================================================

@dataclass
class Reading:
    seq: int
    timestamp_utc: str
    temperature_c: float
    humidity_pct: float
    pressure_hpa: float
    sample_id: Optional[str] = None
    raw_adc: Optional[dict] = None
    injected: bool = False


def utc_now() -> str:
    return dt.datetime.now(dt.timezone.utc).isoformat(timespec="milliseconds")


class Bme280Sensor:
    """Real BME280 on the Raspberry Pi I2C bus (RPi.bme280 + smbus2)."""

    def __init__(self, port: int = I2C_PORT, address: int = I2C_ADDRESS):
        import bme280  # RPi.bme280
        import smbus2

        self._bme280 = bme280
        self.port = port
        self.address = address
        self.bus = smbus2.SMBus(port)
        self.chip_id = self.bus.read_byte_data(address, BME280_CHIP_ID_REG)
        self.calibration = bme280.load_calibration_params(self.bus, address)

    def read(self, seq: int) -> Reading:
        data = self._bme280.sample(self.bus, self.address, self.calibration)
        raw = None
        unc = getattr(data, "uncompensated", None)
        if unc is not None:
            raw = {k: getattr(unc, k, None) for k in ("temperature", "pressure", "humidity")}
        return Reading(
            seq=seq,
            timestamp_utc=utc_now(),
            temperature_c=float(data.temperature),
            humidity_pct=float(data.humidity),
            pressure_hpa=float(data.pressure),
            sample_id=str(getattr(data, "id", "")) or None,
            raw_adc=raw,
        )

    def describe(self) -> dict:
        try:
            calibration = {k: v for k, v in vars(self.calibration).items() if not k.startswith("_")}
        except TypeError:
            calibration = repr(self.calibration)
        return {
            "type": "BME280 (RPi.bme280 driver)",
            "i2c_bus": f"/dev/i2c-{self.port}",
            "i2c_address": f"0x{self.address:02X}",
            "chip_id": f"0x{self.chip_id:02X}",
            "chip_id_ok": self.chip_id == BME280_EXPECTED_CHIP_ID,
            "calibration": calibration,
        }


class SimulatedSensor:
    """Random-walk sensor for rehearsing without hardware."""

    def __init__(self) -> None:
        self.rng = random.Random()
        self.t, self.h, self.p = 22.5, 46.0, 1013.2

    def read(self, seq: int) -> Reading:
        self.t = min(30.0, max(15.0, self.t + self.rng.uniform(-0.15, 0.15)))
        self.h = min(60.0, max(30.0, self.h + self.rng.uniform(-0.6, 0.6)))
        self.p = min(1030.0, max(995.0, self.p + self.rng.uniform(-0.2, 0.2)))
        return Reading(seq, utc_now(), round(self.t, 2), round(self.h, 3), round(self.p, 2))

    def describe(self) -> dict:
        return {"type": "simulated (random walk)"}


# =============================================================================
# Diagnostics collection
# =============================================================================

def _read_text(path: str) -> Optional[str]:
    try:
        return Path(path).read_text(errors="replace").strip("\x00\n ")
    except Exception:
        return None


def _run(cmd: list[str], cwd: Optional[Path] = None) -> Optional[str]:
    try:
        out = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True, timeout=5)
        return out.stdout.strip() if out.returncode == 0 else None
    except Exception:
        return None


def _cpuinfo_field(name: str) -> Optional[str]:
    text = _read_text("/proc/cpuinfo") or ""
    for line in text.splitlines():
        if line.lower().startswith(name.lower()):
            return line.split(":", 1)[-1].strip()
    return None


def _os_pretty_name() -> Optional[str]:
    text = _read_text("/etc/os-release") or ""
    for line in text.splitlines():
        if line.startswith("PRETTY_NAME="):
            return line.split("=", 1)[1].strip('"')
    return None


def _package_versions() -> dict:
    from importlib import metadata

    versions = {}
    for pkg in ("RPi.bme280", "smbus2", "requests"):
        try:
            versions[pkg] = metadata.version(pkg)
        except Exception:
            versions[pkg] = "not installed"
    return versions


def collect_git_info() -> dict:
    here = Path(__file__).resolve().parent
    top = _run(["git", "rev-parse", "--show-toplevel"], here)
    if not top:
        return {"available": False}
    toplevel = Path(top)
    return {
        "available": True,
        "commit": _run(["git", "rev-parse", "HEAD"], here),
        "branch": _run(["git", "rev-parse", "--abbrev-ref", "HEAD"], here),
        "dirty": bool(_run(["git", "status", "--porcelain"], here)),
        "describe": _run(["git", "describe", "--always", "--dirty", "--tags"], here),
        "last_commit": _run(["git", "log", "-1", "--format=%h %s (%an, %ad)", "--date=iso"], here),
        "remote": _run(["git", "remote", "get-url", "origin"], here),
        "toplevel": str(toplevel),
        "script_path": str(Path(__file__).resolve().relative_to(toplevel)),
    }


def collect_static_info() -> dict:
    """Collected once at startup."""
    return {
        "app": {"name": APP_NAME, "version": APP_VERSION, "pid": os.getpid(),
                "argv": sys.argv, "cwd": os.getcwd(), "user": getpass.getuser()},
        "device": {
            "hostname": socket.gethostname(),
            "model": _read_text("/proc/device-tree/model"),
            "serial": _cpuinfo_field("Serial"),
            "revision": _cpuinfo_field("Revision"),
        },
        "os": {"pretty_name": _os_pretty_name(), "kernel": platform.release(),
               "machine": platform.machine()},
        "python": {"version": sys.version.replace("\n", " "), "executable": sys.executable},
        "packages": _package_versions(),
        "git": collect_git_info(),
    }


def collect_dynamic_info(start_monotonic: float) -> dict:
    """Collected at the moment of each incident."""
    info: dict[str, Any] = {"app_uptime_s": round(time.monotonic() - start_monotonic, 1)}
    t = _read_text("/sys/class/thermal/thermal_zone0/temp")
    info["cpu_temp_c"] = round(int(t) / 1000, 1) if t and t.isdigit() else None
    up = _read_text("/proc/uptime")
    info["system_uptime_s"] = float(up.split()[0]) if up else None
    try:
        info["load_avg"] = os.getloadavg()
    except OSError:
        info["load_avg"] = None
    mem = {}
    for line in (_read_text("/proc/meminfo") or "").splitlines():
        key, _, val = line.partition(":")
        if key in ("MemTotal", "MemAvailable"):
            mem[key] = val.strip()
    info["memory"] = mem or None
    try:
        du = shutil.disk_usage("/")
        info["disk_root"] = {"total_gb": round(du.total / 1e9, 1), "free_gb": round(du.free / 1e9, 1)}
    except Exception:
        info["disk_root"] = None
    info["throttled"] = _run(["vcgencmd", "get_throttled"])   # Pi under-voltage / thermal flags
    info["i2c_device_present"] = Path(f"/dev/i2c-{I2C_PORT}").exists()
    return info


_INT_RANGES = {
    "b": (-(2**7), 2**7 - 1), "B": (0, 2**8 - 1),
    "h": (-(2**15), 2**15 - 1), "H": (0, 2**16 - 1),
    "i": (-(2**31), 2**31 - 1), "I": (0, 2**32 - 1),
    "l": (-(2**31), 2**31 - 1), "L": (0, 2**32 - 1),
    "q": (-(2**63), 2**63 - 1), "Q": (0, 2**64 - 1),
}


def frame_field_analysis(reading: Reading) -> list[dict]:
    """For each measurement: scaled wire value, storage type, and whether the
    sensor's full rated range is representable. Derived from FRAME_STRUCT and
    FIELD_SCALES so it stays accurate if the wire format changes."""
    fmt = FRAME_STRUCT.format
    fmt = fmt.decode() if isinstance(fmt, bytes) else fmt
    codes = [c for c in fmt if c.isalpha()]
    code_by_field = dict(zip(FRAME_FIELDS, codes))
    rows = []
    for name, scale in FIELD_SCALES.items():
        value = getattr(reading, name)
        code = code_by_field.get(name, "?")
        lo, hi = _INT_RANGES.get(code, (None, None))
        scaled = round(value * scale)
        rated_lo, rated_hi = SENSOR_RATED_RANGES[name]
        rated_fits = lo is not None and lo <= round(rated_lo * scale) and round(rated_hi * scale) <= hi
        rows.append({
            "field": name,
            "value": value,
            "scale": scale,
            "scaled": scaled,
            "struct_code": code,
            "int_range": f"{lo} .. {hi}",
            "representable_physical": f"{lo / scale:g} .. {hi / scale:g}" if lo is not None else "?",
            "sensor_rated_range": f"{rated_lo:g} .. {rated_hi:g}",
            "value_fits": lo is not None and lo <= scaled <= hi,
            "rated_range_fits": rated_fits,
        })
    return rows


# =============================================================================
# Incident construction
# =============================================================================

def _short_repr(value: str, limit: int = 240) -> str:
    value = value.replace("\n", "\\n")
    return value if len(value) <= limit else value[: limit - 3] + "..."


def _fence(text: str) -> str:
    return "````text\n" + text.replace("````", "'''") + "\n````"


def _md_cell(value: Any) -> str:
    return str(value).replace("|", "\\|").replace("\n", " ")


def build_incident(
    exc: BaseException,
    stage: str,
    reading: Optional[Reading],
    history: deque,
    sensor_info: dict,
    static_info: dict,
    start_monotonic: float,
) -> dict:
    tb_exc = traceback.TracebackException(type(exc), exc, exc.__traceback__, capture_locals=True)
    tb_text = "".join(traceback.format_exception(type(exc), exc, exc.__traceback__))
    frames = list(tb_exc.stack)

    # Fault site = innermost frame in this file (struct.error is raised in C,
    # so the innermost *Python* frame is the line that called pack()).
    this_file = str(Path(__file__).resolve())
    ours = [f for f in frames if str(Path(f.filename).resolve()) == this_file]
    site = ours[-1] if ours else (frames[-1] if frames else None)
    site_func = site.name if site else "?"
    site_line = site.lineno if site else 0

    exc_type = f"{type(exc).__module__}.{type(exc).__qualname__}".replace("builtins.", "")
    exc_msg = str(exc) or "(no message)"
    fp_src = f"{exc_type}|{Path(this_file).name}|{site_func}|{site_line}|{stage}"
    fingerprint = hashlib.sha1(fp_src.encode()).hexdigest()[:12]

    git = static_info.get("git", {})
    device = static_info.get("device", {})
    dynamic = collect_dynamic_info(start_monotonic)
    now = utc_now()

    title = f"[{TRIGGER_LABEL}] {type(exc).__name__}: {exc_msg} in {site_func}() ({Path(this_file).name}:{site_line})"
    if len(title) > 200:
        title = title[:197] + "..."

    # ---- Permalink to the faulting line at the exact deployed commit
    permalink = None
    if git.get("available") and git.get("commit") and GITHUB_REPO != "OWNER/REPO" and site:
        a, b = max(1, site_line - 8), site_line + 4
        permalink = f"https://github.com/{GITHUB_REPO}/blob/{git['commit']}/{git['script_path']}#L{a}-L{b}"

    # ---- Source context
    src_lines = []
    if site:
        for n in range(max(1, site_line - 12), site_line + 5):
            line = linecache.getline(site.filename, n)
            if not line:
                continue
            marker = ">>" if n == site_line else "  "
            src_lines.append(f"{marker} {n:5d} | {line.rstrip()}")
    source_context = "\n".join(src_lines) or "(source unavailable)"

    # ---- Frames with locals
    frame_blocks = []
    for i, fr in enumerate(frames):
        locals_txt = "\n".join(
            f"    {k} = {_short_repr(v)}" for k, v in (fr.locals or {}).items()
        ) or "    (no locals captured)"
        frame_blocks.append(
            f"#{i} {fr.name}()  {fr.filename}:{fr.lineno}\n    > {fr.line}\n{locals_txt}"
        )
    frames_text = "\n\n".join(frame_blocks)

    # ---- Reading at failure + wire-format analysis
    reading_md = "_No reading available (failure occurred before a sample was obtained)._"
    field_md = ""
    if reading is not None:
        reading_md = "\n".join([
            "| Quantity | Value | Raw ADC |",
            "|---|---|---|",
            f"| Temperature | {reading.temperature_c:.2f} °C ({reading.temperature_c * 9 / 5 + 32:.2f} °F) | {(reading.raw_adc or {}).get('temperature', 'n/a')} |",
            f"| Humidity | {reading.humidity_pct:.3f} %RH | {(reading.raw_adc or {}).get('humidity', 'n/a')} |",
            f"| Pressure | {reading.pressure_hpa:.2f} hPa | {(reading.raw_adc or {}).get('pressure', 'n/a')} |",
            f"| Sample seq / id | {reading.seq} / `{reading.sample_id or 'n/a'}` | |",
            f"| Sampled at | {reading.timestamp_utc} | |",
            f"| Fault-injected sample | {'**yes**' if reading.injected else 'no'} | |",
        ])
        rows = frame_field_analysis(reading)
        field_md = "\n".join(
            ["| Field | Value | Scale | Scaled int | Type | Int range | Representable | Sensor rated range | Value fits? | Rated range fits? |",
             "|---|---|---|---|---|---|---|---|---|---|"]
            + [
                f"| `{r['field']}` | {r['value']} | ×{r['scale']} | {r['scaled']} | `{r['struct_code']}` | {r['int_range']} "
                f"| {r['representable_physical']} | {r['sensor_rated_range']} "
                f"| {'✅' if r['value_fits'] else '❌'} | {'✅' if r['rated_range_fits'] else '❌'} |"
                for r in rows
            ]
        )

    # ---- History
    hist_rows = ["| Seq | Time (UTC) | Temp °C | RH % | hPa | Injected |", "|---|---|---|---|---|---|"]
    for h in list(history)[-20:]:
        hist_rows.append(f"| {h.seq} | {h.timestamp_utc} | {h.temperature_c:.2f} | {h.humidity_pct:.3f} | {h.pressure_hpa:.2f} | {'yes' if h.injected else ''} |")
    history_md = "\n".join(hist_rows) if len(hist_rows) > 2 else "_No prior samples._"

    commit_display = (git.get("commit") or "unknown")[:12]
    if git.get("dirty"):
        commit_display += " (dirty working tree)"

    marker = f"<!-- {APP_NAME}:fingerprint={fingerprint} -->"

    sections = [
        marker,
        f"> [!IMPORTANT]\n> Filed automatically by `{APP_NAME}` v{APP_VERSION} running on **{device.get('hostname')}**. "
        f"An unhandled exception occurred in the **`{stage}`** stage. The device caught the fault and **kept running**.",
        "## Summary",
        "\n".join([
            "| | |", "|---|---|",
            f"| Exception | `{_md_cell(exc_type)}` |",
            f"| Message | `{_md_cell(exc_msg)}` |",
            f"| Fault site | `{site_func}()` at `{Path(this_file).name}:{site_line}` |",
            f"| Pipeline stage | `{stage}` |",
            f"| First seen (UTC) | {now} |",
            f"| Device | {_md_cell(device.get('model') or 'unknown')} · serial `{device.get('serial') or 'n/a'}` |",
            f"| Deployed commit | `{commit_display}` on `{git.get('branch') or 'n/a'}` |",
            f"| Fingerprint | `{fingerprint}` |",
            f"| Source at deployed commit | {permalink or '_(not available — repo not configured or not a git checkout)_'} |",
        ]),
        "## Observed vs. expected",
        f"**Observed:** sample #{reading.seq if reading else '?'} raised `{type(exc).__name__}: {exc_msg}` during `{stage}`. "
        "The sample was dropped and no telemetry frame was transmitted.\n\n"
        "**Expected:** every physically valid reading within the BME280's rated operating range "
        "(−40…85 °C, 0…100 %RH, 300…1100 hPa) is encoded, transmitted, and decodes back to the original value.",
        "## Reading at time of failure",
        reading_md,
    ]
    if field_md:
        sections += ["## Telemetry frame field analysis", 
                     f"Wire format `FRAME_STRUCT = struct.Struct({FRAME_STRUCT.format!r})`, `FRAME_VERSION = {FRAME_VERSION}`.\n\n" + field_md]
    sections += [
        "## Stack trace",
        _fence(tb_text.rstrip()),
        "## Source context",
        _fence(source_context),
        "<details><summary><b>Stack frames with local variables</b></summary>\n\n" + _fence(frames_text) + "\n\n</details>",
        f"<details><summary><b>Recent samples (last {min(20, len(history))})</b></summary>\n\n" + history_md + "\n\n</details>",
        "<details><summary><b>Sensor</b></summary>\n\n```json\n" + json.dumps(sensor_info, indent=2, default=str) + "\n```\n\n</details>",
        "<details><summary><b>Device &amp; runtime at time of fault</b></summary>\n\n```json\n" + json.dumps(dynamic, indent=2, default=str) + "\n```\n\n</details>",
        "<details><summary><b>Build &amp; environment</b></summary>\n\n```json\n" + json.dumps(static_info, indent=2, default=str) + "\n```\n\n</details>",
        "## Reproduction",
        "1. Run `sensor_monitor.py` on the device (or `--simulate` on any machine).\n"
        f"2. Produce a reading equal to the one above (for humidity: breathe on the sensor; or `kill -USR1 <pid>` to inject).\n"
        f"3. Unit-level: `encode_frame(1, {reading.temperature_c if reading else 22.0}, {reading.humidity_pct if reading else 50.0}, {reading.pressure_hpa if reading else 1013.0})`.",
        "## Acceptance criteria",
        "- [ ] Root cause fixed — no `try/except` that hides or drops the sample.\n"
        "- [ ] Every value across the sensor's full rated range encodes and round-trips through `decode_frame` within one LSB.\n"
        "- [ ] Regression tests cover the failing value and both ends of every rated range.\n"
        "- [ ] If the wire format changes, `FRAME_VERSION` is incremented.",
    ]
    body = "\n\n".join(sections)

    # Trim progressively if over GitHub's limit
    if len(body) > MAX_ISSUE_BODY_CHARS:
        body = body.replace(_fence(frames_text), "_(frame locals omitted: body size limit)_")
    if len(body) > MAX_ISSUE_BODY_CHARS:
        body = body[: MAX_ISSUE_BODY_CHARS - 200] + "\n\n_…truncated: body size limit._\n\n" + marker

    summary = (
        f"`{type(exc).__name__}: {exc_msg}` at {now}"
        + (f" — T={reading.temperature_c:.2f} °C, RH={reading.humidity_pct:.3f} %, P={reading.pressure_hpa:.2f} hPa"
           f"{' (injected)' if reading.injected else ''}" if reading else "")
    )

    return {
        "fingerprint": fingerprint,
        "title": title,
        "body": body,
        "labels": ISSUE_LABELS,
        "captured_at": now,
        "summary": summary,
        "commit": git.get("commit"),
    }


# =============================================================================
# GitHub issue reporter (background thread, never blocks the sampling loop)
# =============================================================================

class TransientError(Exception):
    pass


class GitHubIssueReporter:
    def __init__(self, repo: str, token: Optional[str], dry_run: bool, logger: logging.Logger, console):
        self.repo = repo
        self.token = token
        self.dry_run = dry_run or not token or repo == "OWNER/REPO"
        self.log = logger
        self.console = console
        self._q: "queue.Queue[Optional[dict]]" = queue.Queue(maxsize=200)
        self._known: dict[str, dict] = {}
        self._labels_ready = False
        self._session = None
        self._thread = threading.Thread(target=self._run, name="issue-reporter", daemon=True)
        SPOOL_DIR.mkdir(parents=True, exist_ok=True)

    # ---- public API (called from the sampling thread)
    def start(self) -> None:
        self._thread.start()

    def report(self, incident: dict) -> None:
        try:
            self._q.put_nowait(incident)
        except queue.Full:
            self.log.error("Reporter queue full; dropping incident %s", incident.get("fingerprint"))

    def stop(self, timeout: float = 15.0) -> None:
        self._q.put(None)
        self._thread.join(timeout)

    # ---- worker
    def _run(self) -> None:
        last_spool_retry = 0.0
        while True:
            try:
                item = self._q.get(timeout=5)
            except queue.Empty:
                item = "idle"
            if item is None:
                self._flush_recurrences(force=True)
                return
            try:
                if isinstance(item, dict):
                    self._handle(item)
                self._flush_recurrences()
                if time.monotonic() - last_spool_retry > SPOOL_RETRY_INTERVAL_S:
                    last_spool_retry = time.monotonic()
                    self._retry_spool()
            except Exception:
                self.log.exception("Reporter worker error (continuing)")

    def _handle(self, incident: dict) -> None:
        fp = incident["fingerprint"]
        known = self._known.get(fp)
        if known:
            known["pending"] += 1
            known["latest"] = incident
            return

        if self.dry_run:
            path = SPOOL_DIR / f"dry-run-{fp}.md"
            path.write_text(f"# {incident['title']}\n\n{incident['body']}\n")
            self._known[fp] = {"number": None, "pending": 0, "last_comment": time.monotonic(), "latest": incident}
            self.console(f"DRY RUN: issue body written to {path}", "yellow")
            return

        try:
            self._ensure_labels()
            existing = self._find_open_issue(fp)
            if existing:
                number, url = existing
                self._comment(number, f"### 🔁 Fault recurred\n{incident['summary']}\n\nCommit: `{(incident.get('commit') or 'unknown')[:12]}`")
                self.console(f"Fault {fp} matches open issue #{number} — added recurrence comment: {url}", "yellow")
            else:
                number, url = self._create_issue(incident)
                self.console(f"Filed issue #{number}: {url}", "green")
            self._known[fp] = {"number": number, "pending": 0, "last_comment": time.monotonic(), "latest": incident}
        except TransientError as e:
            self._spool(incident, str(e))
        except Exception as e:
            self.log.exception("Failed to report incident %s", fp)
            self._spool(incident, repr(e))

    def _flush_recurrences(self, force: bool = False) -> None:
        if self.dry_run:
            return
        now = time.monotonic()
        for fp, k in self._known.items():
            if not k["pending"] or k["number"] is None:
                continue
            if force or now - k["last_comment"] >= RECURRENCE_COMMENT_COOLDOWN_S:
                try:
                    self._comment(
                        k["number"],
                        f"### 🔁 Recurrence update\n**{k['pending']}** additional occurrence(s) since the last update.\n\n"
                        f"Latest: {k['latest']['summary']}",
                    )
                    k["pending"] = 0
                    k["last_comment"] = now
                except Exception:
                    self.log.exception("Recurrence comment failed for %s", fp)

    # ---- spool (offline resilience)
    def _spool(self, incident: dict, reason: str) -> None:
        path = SPOOL_DIR / f"{int(time.time() * 1000)}-{incident['fingerprint']}.json"
        path.write_text(json.dumps(incident))
        self.console(f"GitHub unreachable ({reason}); incident spooled to {path}", "yellow")

    def _retry_spool(self) -> None:
        if self.dry_run:
            return
        for path in sorted(SPOOL_DIR.glob("*.json")):
            try:
                incident = json.loads(path.read_text())
            except Exception:
                path.rename(path.with_suffix(".bad"))
                continue
            path.unlink(missing_ok=True)
            self._handle(incident)   # re-spools itself if still failing
            if any(SPOOL_DIR.glob(f"*-{incident['fingerprint']}.json")):
                break                # still offline; try later

    # ---- GitHub REST
    def _api(self, method: str, path: str, **kwargs):
        import requests

        if self._session is None:
            self._session = requests.Session()
            self._session.headers.update({
                "Authorization": f"Bearer {self.token}",
                "Accept": "application/vnd.github+json",
                "X-GitHub-Api-Version": "2022-11-28",
                "User-Agent": f"{APP_NAME}/{APP_VERSION}",
            })
        try:
            r = self._session.request(method, f"{GITHUB_API}{path}", timeout=15, **kwargs)
        except requests.RequestException as e:
            raise TransientError(f"network error: {e}") from e
        if r.status_code >= 500 or (r.status_code in (403, 429) and r.headers.get("x-ratelimit-remaining") == "0"):
            raise TransientError(f"HTTP {r.status_code}")
        if r.status_code in (401, 403):
            raise RuntimeError(f"GitHub auth/permission error HTTP {r.status_code}: {r.text[:300]}")
        return r

    def _ensure_labels(self) -> None:
        if self._labels_ready:
            return
        for name in ISSUE_LABELS:
            r = self._api("GET", f"/repos/{self.repo}/labels/{quote(name)}")
            if r.status_code == 404:
                color, desc = LABEL_DEFINITIONS.get(name, ("ededed", ""))
                self._api("POST", f"/repos/{self.repo}/labels", json={"name": name, "color": color, "description": desc})
        self._labels_ready = True

    def _find_open_issue(self, fingerprint: str) -> Optional[tuple[int, str]]:
        marker = f"{APP_NAME}:fingerprint={fingerprint}"
        r = self._api("GET", f"/repos/{self.repo}/issues",
                      params={"state": "open", "labels": TRIGGER_LABEL, "per_page": 100})
        r.raise_for_status()
        for item in r.json():
            if "pull_request" not in item and marker in (item.get("body") or ""):
                return item["number"], item["html_url"]
        return None

    def _create_issue(self, incident: dict) -> tuple[int, str]:
        r = self._api("POST", f"/repos/{self.repo}/issues",
                      json={"title": incident["title"], "body": incident["body"], "labels": incident["labels"]})
        if r.status_code != 201:
            raise RuntimeError(f"Create issue failed HTTP {r.status_code}: {r.text[:500]}")
        data = r.json()
        return data["number"], data["html_url"]

    def _comment(self, number: int, body: str) -> None:
        r = self._api("POST", f"/repos/{self.repo}/issues/{number}/comments", json={"body": body})
        if r.status_code != 201:
            raise RuntimeError(f"Comment failed HTTP {r.status_code}: {r.text[:300]}")


# =============================================================================
# Console + main loop
# =============================================================================

_COLORS = {"red": "31", "green": "32", "yellow": "33", "cyan": "36", "dim": "2", "bold": "1"}
_console_lock = threading.Lock()


def console(msg: str, color: Optional[str] = None) -> None:
    if color and sys.stdout.isatty():
        msg = f"\033[{_COLORS[color]}m{msg}\033[0m"
    with _console_lock:
        print(msg, flush=True)


def setup_logging() -> logging.Logger:
    STATE_DIR.mkdir(parents=True, exist_ok=True)
    logger = logging.getLogger(APP_NAME)
    logger.setLevel(logging.DEBUG)
    fh = logging.handlers.RotatingFileHandler(LOG_FILE, maxBytes=1_000_000, backupCount=3)
    fh.setFormatter(logging.Formatter("%(asctime)s %(levelname)s [%(threadName)s] %(message)s"))
    logger.addHandler(fh)
    eh = logging.StreamHandler(sys.stderr)
    eh.setLevel(logging.ERROR)
    logger.addHandler(eh)
    return logger


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--simulate", action="store_true", help="use a simulated sensor (no hardware)")
    ap.add_argument("--dry-run", action="store_true", help="write issues to disk instead of GitHub")
    ap.add_argument("--interval", type=float, default=SAMPLE_INTERVAL_S, help="seconds between samples")
    ap.add_argument("--inject-humidity", type=float, default=72.5,
                    help="humidity value used when a fault is injected via SIGUSR1")
    args = ap.parse_args()

    log = setup_logging()
    start_monotonic = time.monotonic()
    stop = threading.Event()
    inject = threading.Event()

    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    if hasattr(signal, "SIGUSR1"):
        signal.signal(signal.SIGUSR1, lambda *_: inject.set())

    static_info = collect_static_info()
    token = os.environ.get(GITHUB_TOKEN_ENV)
    reporter = GitHubIssueReporter(GITHUB_REPO, token, args.dry_run, log, console)
    reporter.start()

    history: deque[Reading] = deque(maxlen=HISTORY_LEN)
    sensor = None
    sensor_info: dict = {}
    seq = 0

    console(f"{APP_NAME} v{APP_VERSION}  pid={os.getpid()}  commit={(static_info['git'].get('commit') or 'n/a')[:12]}", "bold")
    console(f"Reporting to: {'DRY RUN (' + str(SPOOL_DIR) + ')' if reporter.dry_run else GITHUB_REPO}"
            f"   Inject a fault: kill -USR1 {os.getpid()}   Stop: Ctrl+C\n", "dim")

    def report(exc: BaseException, stage: str, reading: Optional[Reading]) -> None:
        log.error("Fault in %s: %r", stage, exc, exc_info=exc)
        try:
            incident = build_incident(exc, stage, reading, history, sensor_info, static_info, start_monotonic)
            reporter.report(incident)
            console(f"⚠  FAULT [{stage}] {type(exc).__name__}: {exc}  (fingerprint {incident['fingerprint']}) — reported, continuing", "red")
        except Exception:
            # The diagnostics path must never take the device down either.
            log.exception("Failed to build incident report")
            console(f"⚠  FAULT [{stage}] {type(exc).__name__}: {exc} — report generation failed, see {LOG_FILE}", "red")

    while not stop.is_set():
        reading: Optional[Reading] = None
        try:
            # ---- Stage 1: sensor (re)initialisation
            if sensor is None:
                try:
                    sensor = SimulatedSensor() if args.simulate else Bme280Sensor()
                    sensor_info = sensor.describe()
                    console(f"Sensor ready: {sensor_info.get('type')} "
                            f"{sensor_info.get('i2c_address', '')} chip_id={sensor_info.get('chip_id', 'n/a')}", "cyan")
                except Exception as e:
                    sensor = None
                    report(e, "sensor_init", None)
                    stop.wait(5)
                    continue

            # ---- Stage 2: sample
            seq += 1
            try:
                reading = sensor.read(seq)
            except Exception as e:
                report(e, "sensor_read", None)
                sensor = None          # force re-init on next loop (e.g. I2C glitch)
                stop.wait(args.interval)
                continue

            if inject.is_set():
                inject.clear()
                reading.humidity_pct = args.inject_humidity
                reading.injected = True

            history.append(reading)
            temp_f = reading.temperature_c * 9 / 5 + 32
            console(f"#{reading.seq:<5} Temperature: {reading.temperature_c:6.2f} °C ({temp_f:6.2f} °F)   "
                    f"Humidity: {reading.humidity_pct:6.2f} %   Pressure: {reading.pressure_hpa:7.2f} hPa"
                    + ("   [injected]" if reading.injected else ""))

            # ---- Stage 3: encode + transmit
            try:
                frame = encode_frame(reading.seq, reading.temperature_c, reading.humidity_pct, reading.pressure_hpa)
                transmit_frame(frame)
            except Exception as e:
                report(e, "telemetry_encode", reading)

        except Exception as e:  # anything unforeseen in the loop itself
            report(e, "main_loop", reading)

        stop.wait(args.interval)

    console("\nStopping — flushing pending reports…", "dim")
    reporter.stop()
    console("Program stopped.", "dim")
    return 0


if __name__ == "__main__":
    sys.exit(main())
