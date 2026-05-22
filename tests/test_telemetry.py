"""
Tests for protocol/telemetry.py.
No network required — packets are constructed via packet.build().
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import struct
import pytest
from process.protocol.packet import build
from process.protocol.telemetry import (
    parse_telemetry, decode_voltage,
    BatteryData, GpsAttitude, FlightData,
    FollowMeStatus, WaypointStatus, DroneInfo,
    SettingsStatus, AlertData, DroneStatus,
    PhotoNotification, VideoNotification, CloseAlert,
)


def _pkt(cmd: int, payload: bytes) -> bytes:
    """Build a valid raw buffer for a given cmd+payload."""
    return build(cmd, payload)


class TestBatteryParsing:
    def test_basic(self):
        result = parse_telemetry(_pkt(0x00, bytes([0x78, 0x64])))
        assert isinstance(result, BatteryData)
        assert result.drone_raw == 0x78
        assert result.rc_raw == 0x64

    def test_voltage_decoding(self):
        assert decode_voltage(120) == 12.0
        assert decode_voltage(90) == 9.0
        assert decode_voltage(75) == 7.5

    def test_f11_threshold(self):
        # > 9.0V (raw > 90) = F11 3S battery
        assert decode_voltage(91) > 9.0
        assert decode_voltage(90) == 9.0


class TestGpsParsing:
    def _encode(self, lon, lat, signal=3, heading=180):
        payload = struct.pack("<ii", int(lon * 1e7), int(lat * 1e7))
        payload += bytes([signal, (heading >> 8) & 0xFF, heading & 0xFF])
        return payload

    def test_coordinates(self):
        result = parse_telemetry(_pkt(0x01, self._encode(-97.7431, 30.2672)))
        assert isinstance(result, GpsAttitude)
        assert abs(result.longitude - (-97.7431)) < 1e-5
        assert abs(result.latitude  -  30.2672)  < 1e-5

    def test_heading(self):
        result = parse_telemetry(_pkt(0x01, self._encode(0.0, 0.0, heading=270)))
        assert result.heading == 270

    def test_signal(self):
        for sig in range(4):
            result = parse_telemetry(_pkt(0x01, self._encode(0.0, 0.0, signal=sig)))
            assert result.gps_signal == sig

    def test_tilt_present(self):
        result = parse_telemetry(_pkt(0x01, self._encode(0.0, 0.0) + bytes([10, 5])))
        assert result.tilt_fore_aft == 10
        assert result.tilt_left_right == 5

    def test_tilt_absent(self):
        result = parse_telemetry(_pkt(0x01, self._encode(0.0, 0.0)))
        assert result.tilt_fore_aft is None
        assert result.tilt_left_right is None


class TestFlightDataParsing:
    def _encode(self, dist, alt, hspd, vspd, status=0x00):
        return (
            struct.pack(">hhhh", int(dist*10), int(alt*10), int(hspd*10), int(vspd*10))
            + bytes([status])
        )

    def test_basic(self):
        result = parse_telemetry(_pkt(0x02, self._encode(15.3, 22.5, 3.1, -0.5)))
        assert isinstance(result, FlightData)
        assert abs(result.distance_m     - 15.3) < 0.01
        assert abs(result.altitude_m     - 22.5) < 0.01
        assert abs(result.speed_horiz_ms -  3.1) < 0.01

    def test_negative_values_clamped(self):
        result = parse_telemetry(_pkt(0x02, self._encode(-5.0, -2.0, -1.0, -1.0)))
        assert result.distance_m     == 0.0
        assert result.altitude_m     == 0.0
        assert result.speed_horiz_ms == 0.0

    def test_invalid_status(self):
        result = parse_telemetry(_pkt(0x02, self._encode(0, 0, 0, 0, status=0x0F)))
        assert result.status == 0x0F


class TestNotifications:
    def test_photo(self):
        assert isinstance(parse_telemetry(_pkt(0x03, b"")), PhotoNotification)

    def test_video(self):
        assert isinstance(parse_telemetry(_pkt(0x04, b"")), VideoNotification)

    def test_close_alert(self):
        assert isinstance(parse_telemetry(_pkt(0x06, b"")), CloseAlert)


class TestAlertParsing:
    def test_gbk_string(self):
        msg = "低电量警告"
        result = parse_telemetry(_pkt(0x05, msg.encode("gbk")))
        assert isinstance(result, AlertData)
        assert result.message == msg


class TestFollowMeStatus:
    def test_active(self):
        result = parse_telemetry(_pkt(0x07, bytes([0x01])))
        assert isinstance(result, FollowMeStatus)
        assert result.active is True

    def test_cancelled(self):
        result = parse_telemetry(_pkt(0x07, bytes([0x00])))
        assert result.active is False


class TestWaypointStatus:
    def test_each_code(self):
        for code in [0x01, 0x02, 0x03, 0x04, 0x05]:
            result = parse_telemetry(_pkt(0x08, bytes([code])))
            assert isinstance(result, WaypointStatus)
            assert result.code == code


class TestDroneInfo:
    def test_strips_sj19_prefix(self):
        result = parse_telemetry(_pkt(0x09, "SJ19F11-GPS v1.0".encode("gbk")))
        assert isinstance(result, DroneInfo)
        assert result.raw == "F11-GPS v1.0"

    def test_no_prefix(self):
        result = parse_telemetry(_pkt(0x09, "F11-GPS".encode("gbk")))
        assert result.raw == "F11-GPS"


class TestSettingsStatus:
    def test_all_codes(self):
        for code in [0x00, 0x01, 0x02, 0x03, 0x08, 0x0F]:
            result = parse_telemetry(_pkt(0x0B, bytes([code])))
            assert isinstance(result, SettingsStatus)
            assert result.code == code


class TestDroneStatus:
    def test_normal_code(self):
        result = parse_telemetry(_pkt(0x0C, bytes([0x02])))
        assert isinstance(result, DroneStatus)
        assert result.code == 2

    def test_dismiss_ff(self):
        result = parse_telemetry(_pkt(0x0C, bytes([0xFF])))
        assert result.code == -1


class TestInvalidPackets:
    def test_bad_magic(self):
        assert parse_telemetry(bytes([0x00, 0x00, 0x01, 0x00, 0x00])) is None

    def test_unknown_command(self):
        assert parse_telemetry(_pkt(0xFE, bytes([0x00]))) is None

    def test_truncated(self):
        assert parse_telemetry(bytes([0x5A, 0x55])) is None