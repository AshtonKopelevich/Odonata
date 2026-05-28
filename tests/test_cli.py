"""
Tests for cli.py.

Tests cover display helpers and command dispatch.
The REPL loop and argparse entry point are not tested here — those
are integration concerns better verified manually against a real drone.
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import time
from io import StringIO
from unittest.mock import MagicMock, patch

from cli import (
    print_status, _fmt_voltage, _fmt_gps, _fmt_float,
    _cmd_status, _cmd_takeoff, _cmd_land, _cmd_rth,
    _cmd_cancel_rth, _cmd_stop, _cmd_photo, _cmd_rec, _cmd_help,
)


# ---------------------------------------------------------------------------
# Formatting helpers
# ---------------------------------------------------------------------------

class TestFmtVoltage:
    def test_none(self):
        assert _fmt_voltage(None) == "—"

    def test_value(self):
        assert _fmt_voltage(120) == "12.0V"

    def test_low(self):
        assert _fmt_voltage(75) == "7.5V"


class TestFmtGps:
    def test_none(self):
        assert _fmt_gps(None) == "—"

    def test_zero(self):
        result = _fmt_gps(0)
        assert "0/3" in result

    def test_three(self):
        result = _fmt_gps(3)
        assert "3/3" in result
        assert "█" in result


class TestFmtFloat:
    def test_none(self):
        assert _fmt_float(None, "m") == "—"

    def test_basic(self):
        assert _fmt_float(12.345, "m") == "12.3m"

    def test_zero_decimals(self):
        assert _fmt_float(180.0, "°", 0) == "180°"


class TestPrintStatus:
    def _full_state(self, **overrides) -> dict:
        base = {
            "model":             "F11-GPS",
            "battery_drone_raw": 120,
            "battery_rc_raw":    100,
            "gps_signal":        3,
            "latitude":          30.2672,
            "longitude":         -97.7431,
            "heading":           180,
            "altitude_m":        22.5,
            "distance_m":        15.3,
            "speed_horiz_ms":    3.1,
            "speed_vert_ms":     0.0,
        }
        return {**base, **overrides}

    def test_model_shown(self, capsys):
        print_status(self._full_state())
        out = capsys.readouterr().out
        assert "F11-GPS" in out

    def test_voltage_shown(self, capsys):
        print_status(self._full_state())
        out = capsys.readouterr().out
        assert "12.0V" in out

    def test_none_values_show_dash(self, capsys):
        print_status(self._full_state(altitude_m=None, latitude=None))
        out = capsys.readouterr().out
        assert "—" in out

    def test_all_fields_present(self, capsys):
        print_status(self._full_state())
        out = capsys.readouterr().out
        for label in ["Model", "Battery", "GPS", "Position", "Heading",
                      "Altitude", "Distance", "Speed H", "Speed V"]:
            assert label in out, f"Missing field: {label}"


# ---------------------------------------------------------------------------
# Command handlers
# ---------------------------------------------------------------------------

def _mock_drone(**state_overrides) -> MagicMock:
    drone = MagicMock()
    default_state = {
        "model": "F11-GPS", "battery_drone_raw": 120, "battery_rc_raw": 100,
        "gps_signal": 3, "latitude": 30.0, "longitude": -97.0,
        "heading": 90, "altitude_m": 10.0, "distance_m": 5.0,
        "speed_horiz_ms": 1.0, "speed_vert_ms": 0.0,
    }
    drone.get_state.return_value = {**default_state, **state_overrides}
    return drone


class TestStatusCommand:
    def test_calls_get_state(self, capsys):
        drone = _mock_drone()
        _cmd_status(drone, [])
        drone.get_state.assert_called_once()

    def test_output_contains_model(self, capsys):
        drone = _mock_drone()
        _cmd_status(drone, [])
        assert "F11-GPS" in capsys.readouterr().out


class TestFlightCommands:
    def test_takeoff(self, capsys):
        drone = _mock_drone()
        _cmd_takeoff(drone, [])
        drone.takeoff.assert_called_once()

    def test_land(self, capsys):
        drone = _mock_drone()
        _cmd_land(drone, [])
        drone.land.assert_called_once()

    def test_rth(self, capsys):
        drone = _mock_drone()
        _cmd_rth(drone, [])
        drone.rth.assert_called_once()

    def test_cancel_rth(self, capsys):
        drone = _mock_drone()
        _cmd_cancel_rth(drone, [])
        drone.cancel_rth.assert_called_once()

    def test_stop_confirmed(self, capsys):
        drone = _mock_drone()
        with patch("builtins.input", return_value="y"):
            _cmd_stop(drone, [])
        drone.emergency_stop.assert_called_once()

    def test_stop_cancelled(self, capsys):
        drone = _mock_drone()
        with patch("builtins.input", return_value="n"):
            _cmd_stop(drone, [])
        drone.emergency_stop.assert_not_called()


class TestPhotoCommand:
    def test_saves_to_given_path(self, tmp_path):
        drone = _mock_drone()
        drone.capture_photo.return_value = b"\xFF\xD8\xFF\xE0" + b"\x00" * 10
        out_file = str(tmp_path / "test.jpg")
        _cmd_photo(drone, [out_file])
        assert os.path.exists(out_file)
        with open(out_file, "rb") as f:
            assert f.read()[:2] == b"\xFF\xD8"

    def test_default_filename_is_timestamped(self, tmp_path, capsys):
        drone = _mock_drone()
        drone.capture_photo.return_value = b"\xFF\xD8\xFF\xE0"
        with patch("os.getcwd", return_value=str(tmp_path)):
            with patch("builtins.open", create=True) as mock_open:
                mock_open.return_value.__enter__ = lambda s: s
                mock_open.return_value.__exit__ = MagicMock(return_value=False)
                mock_open.return_value.write = MagicMock()
                _cmd_photo(drone, [])
                path_used = mock_open.call_args.args[0]
                assert path_used.startswith("photo_")
                assert path_used.endswith(".jpg")

    def test_connection_error_shown(self, capsys):
        drone = _mock_drone()
        drone.capture_photo.side_effect = ConnectionError("not connected")
        _cmd_photo(drone, ["out.jpg"])
        assert "Error" in capsys.readouterr().out

    def test_value_error_shown(self, capsys):
        drone = _mock_drone()
        drone.capture_photo.side_effect = ValueError("bad header")
        _cmd_photo(drone, ["out.jpg"])
        assert "Error" in capsys.readouterr().out


class TestRecCommand:
    def test_start(self, capsys):
        drone = _mock_drone()
        _cmd_rec(drone, ["start"])
        drone.start_recording.assert_called_once()

    def test_stop(self, capsys):
        drone = _mock_drone()
        _cmd_rec(drone, ["stop"])
        drone.stop_recording.assert_called_once()

    def test_invalid_subcommand(self, capsys):
        drone = _mock_drone()
        _cmd_rec(drone, ["pause"])
        out = capsys.readouterr().out
        assert "Usage" in out
        drone.start_recording.assert_not_called()
        drone.stop_recording.assert_not_called()

    def test_no_args(self, capsys):
        drone = _mock_drone()
        _cmd_rec(drone, [])
        assert "Usage" in capsys.readouterr().out


class TestHelpCommand:
    def test_lists_all_commands(self, capsys):
        _cmd_help(MagicMock(), [])
        out = capsys.readouterr().out
        for name in ["status", "takeoff", "land", "rth", "stop", "photo", "rec"]:
            assert name in out