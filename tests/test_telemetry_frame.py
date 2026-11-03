import itertools
import struct

import pytest

from sensor_monitor import (
    CRC_STRUCT,
    FIELD_SCALES,
    FRAME_MAGIC,
    FRAME_STRUCT,
    FRAME_VERSION,
    SENSOR_RATED_RANGES,
    crc16_ccitt,
    decode_frame,
    encode_frame,
)


def assert_round_trip(temperature_c, humidity_pct, pressure_hpa):
    reading = dict(
        temperature_c=temperature_c,
        humidity_pct=humidity_pct,
        pressure_hpa=pressure_hpa,
    )
    decoded = decode_frame(encode_frame(24, **reading))
    assert decoded["sequence"] == 24
    assert decoded["version"] == FRAME_VERSION
    for field, value in reading.items():
        assert abs(decoded[field] - value) <= 1 / FIELD_SCALES[field]


def test_reported_reading():
    assert_round_trip(23.27340639689937, 72.5, 983.6407854877274)


@pytest.mark.parametrize(
    "temperature_c,humidity_pct,pressure_hpa",
    list(itertools.product(*SENSOR_RATED_RANGES.values())),
)
def test_rated_boundaries(temperature_c, humidity_pct, pressure_hpa):
    assert_round_trip(temperature_c, humidity_pct, pressure_hpa)


@pytest.mark.parametrize("humidity_pct", [65.534, 65.535, 65.536])
def test_previous_humidity_limit(humidity_pct):
    assert_round_trip(23.27, humidity_pct, 983.64)


@pytest.mark.parametrize("field", SENSOR_RATED_RANGES)
def test_round_trips_across_rated_range(field):
    reading = dict(temperature_c=23.27, humidity_pct=72.5, pressure_hpa=983.64)
    low, high = SENSOR_RATED_RANGES[field]
    for step in range(1001):
        reading[field] = low + (high - low) * step / 1000
        assert_round_trip(**reading)


def test_version_2_wire_layout():
    frame = encode_frame(24, 23.27340639689937, 72.5, 983.6407854877274)
    payload = struct.pack("<BBIhIH", 0xB2, 2, 24, 2327, 72500, 9836)
    assert FRAME_MAGIC == 0xB2
    assert FRAME_VERSION == 2
    assert FRAME_STRUCT.size + CRC_STRUCT.size == 16
    assert frame == payload + struct.pack("<H", crc16_ccitt(payload))
    assert decode_frame(frame)["humidity_pct"] == 72.5


@pytest.mark.parametrize("seq", [0, 2**32 - 1, 2**32])
def test_sequence_wrap(seq):
    decoded = decode_frame(encode_frame(seq, -40, 100, 1100))
    assert decoded["sequence"] == seq & 0xFFFFFFFF


def test_corrupt_frame_rejected():
    frame = bytearray(encode_frame(24, 23.27, 72.5, 983.64))
    frame[8] ^= 1
    with pytest.raises(ValueError, match="CRC mismatch"):
        decode_frame(bytes(frame))


def test_version_1_frame_rejected():
    payload = struct.pack("<BBIhHH", 0xB2, 1, 24, 2327, 45123, 9836)
    frame = payload + struct.pack("<H", crc16_ccitt(payload))
    with pytest.raises(ValueError, match="frame length"):
        decode_frame(frame)
