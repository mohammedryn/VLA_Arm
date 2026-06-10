import struct
import numpy as np
import pytest

from firmware.tools.sensor_listener import (
    PACKET_SIZE,
    SENSOR_MAGIC,
    TOF_MAX_RANGE_M,
    compute_checksum,
    normalize_load,
    parse_packet,
)

BODY_FMT = '<IIBfB3f3f64H'  # everything except the trailing checksum


def _build_packet(*, timestamp_us=0, contact_flag=0, contact_rms=0.0, estop_active=0,
                   accel=(0.0, 0.0, 0.0), gyro=(0.0, 0.0, 0.0), tof_mm=None) -> bytes:
    if tof_mm is None:
        tof_mm = [1000] * 64  # 1m everywhere
    body = struct.pack(
        BODY_FMT,
        SENSOR_MAGIC, timestamp_us, contact_flag, contact_rms, estop_active,
        *accel, *gyro, *tof_mm,
    )
    checksum = sum(body) & 0xFFFF
    return body + struct.pack('<H', checksum)


def test_packet_size():
    assert PACKET_SIZE == 168
    assert len(_build_packet()) == 168


def test_compute_checksum_matches_packet():
    raw = _build_packet(contact_rms=1.5)
    assert compute_checksum(raw[:-2]) == int.from_bytes(raw[-2:], 'little')


def test_parse_packet_valid_returns_expected_fields():
    raw = _build_packet(
        timestamp_us=12345,
        contact_flag=1,
        contact_rms=2.5,
        estop_active=0,
        accel=(0.1, 0.2, 9.8),
        gyro=(1.0, -1.0, 0.5),
    )
    result = parse_packet(raw)
    assert result["timestamp_us"] == 12345
    assert result["contact_flag"] is True
    assert result["estop_active"] is False
    assert result["contact_rms"] == pytest.approx(2.5)
    np.testing.assert_allclose(result["imu"], [0.1, 0.2, 9.8, 1.0, -1.0, 0.5], atol=1e-5)
    assert result["tof"].shape == (8, 8)
    np.testing.assert_allclose(result["tof"], 1.0, atol=1e-5)  # 1000mm -> 1.0m


def test_parse_packet_invalid_tof_zone_clamps_to_max_range():
    tof_mm = [1000] * 64
    tof_mm[0] = 0xFFFF  # invalid sentinel
    raw = _build_packet(tof_mm=tof_mm)
    result = parse_packet(raw)
    assert result["tof"][0, 0] == pytest.approx(TOF_MAX_RANGE_M)


def test_parse_packet_bad_checksum_raises():
    raw = _build_packet()
    corrupted = raw[:-1] + bytes([raw[-1] ^ 0xFF])
    with pytest.raises(ValueError, match="checksum"):
        parse_packet(corrupted)


def test_parse_packet_wrong_size_raises():
    with pytest.raises(ValueError, match="168"):
        parse_packet(b"\x00" * 100)


@pytest.mark.parametrize("raw,expected", [
    (0,    0.0),
    (500,  0.5),
    (1000, 1.0),
    (0x400 | 500, -0.5),  # direction bit set
])
def test_normalize_load(raw, expected):
    assert normalize_load(raw) == pytest.approx(expected)
