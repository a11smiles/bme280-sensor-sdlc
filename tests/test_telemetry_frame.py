import itertools
import struct

import pytest

from sensor_monitor import (
    CRC_STRUCT,
    FIELD_SCALES,
    FRAME_MAGIC,
    FRAME_STRUCT,
    FRAME_VERSION,
    crc16_ccitt,
    decode_frame,
    encode_frame,
)


SCALES = {"temperature_c": 100, "humidity_pct": 1000, "pressure_hpa": 10}
RANGES = {"temperature_c": (-40, 85), "humidity_pct": (0, 100), "pressure_hpa": (300, 1100)}


def assert_round_trip(temperature_c, humidity_pct, pressure_hpa):
    values = dict(zip(SCALES, (temperature_c, humidity_pct, pressure_hpa)))
    frame = encode_frame(16, **values)
    decoded = decode_frame(frame)
    assert decoded["version"] == 2
    assert decoded["sequence"] == 16
    for field, value in values.items():
        assert decoded[field] == pytest.approx(value, abs=1 / SCALES[field], rel=0)
    return frame


def test_reported_reading():
    frame = assert_round_trip(31.074510013504188, 74.99588273345827, 990.2099351607704)
    assert decode_frame(frame)["humidity_pct"] == 74.996


@pytest.mark.parametrize(
    "temperature_c,humidity_pct,pressure_hpa",
    itertools.product((-40, 23.475, 85), (0, 45.1235, 100), (300, 1013.25, 1100)),
)
def test_rated_boundaries_and_rounding(temperature_c, humidity_pct, pressure_hpa):
    assert_round_trip(temperature_c, humidity_pct, pressure_hpa)


@pytest.mark.parametrize("humidity_pct", (65.534, 65.535, 65.5354, 65.5356, 65.536))
def test_previous_humidity_limit(humidity_pct):
    assert_round_trip(23.47, humidity_pct, 1013.2)


@pytest.mark.parametrize("field", SCALES)
def test_every_rated_fixed_point_value(field):
    values = {"temperature_c": 23.47, "humidity_pct": 45.123, "pressure_hpa": 1013.2}
    low, high = RANGES[field]
    for scaled in range(low * SCALES[field], high * SCALES[field] + 1):
        values[field] = scaled / SCALES[field]
        assert_round_trip(**values)


def test_version_2_wire_format():
    assert FRAME_VERSION == 2
    assert FIELD_SCALES == SCALES
    assert FRAME_STRUCT.format == "<BBIhIH"
    frame = encode_frame(16, -40, 100, 1100)
    payload = struct.pack("<BBIhIH", FRAME_MAGIC, 2, 16, -4000, 100000, 11000)
    assert len(frame) == 16
    assert frame == payload + CRC_STRUCT.pack(crc16_ccitt(payload))


def test_legacy_frame_is_rejected():
    payload = struct.pack("<BBIhHH", FRAME_MAGIC, 1, 16, 2347, 45123, 10132)
    frame = payload + CRC_STRUCT.pack(crc16_ccitt(payload))
    with pytest.raises(ValueError, match="frame length"):
        decode_frame(frame)


def test_corrupted_humidity_is_rejected():
    frame = bytearray(encode_frame(16, 23.47, 100, 1013.2))
    frame[10] ^= 1
    with pytest.raises(ValueError, match="CRC mismatch"):
        decode_frame(frame)
