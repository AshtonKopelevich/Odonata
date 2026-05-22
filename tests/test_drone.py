"""
Tests for drone.py.

UDPSocket and TCPSocket are fully mocked so no network is required.
The DroneInfo packet is injected directly into _handle_udp to simulate
the drone's identify response.
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import time
import threading
from unittest.mock import MagicMock, patch, call
import pytest

from process.drone import Drone, DroneState, _local_ip
from process.protocol.packet import build
from process.protocol.telemetry import (
    BatteryData, GpsAttitude, FlightData, DroneInfo,
    PhotoNotification,
)
from process.protocol import commands as cmd


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def _make_drone(on_telemetry=None) -> tuple[Drone, MagicMock, MagicMock]:
    """
    Return a Drone with mocked UDP and TCP layers.
    The caller still needs to call drone.connect() after setting up the mocks
    if they want the full startup sequence, or drive _handle_udp directly.
    """
    udp_mock = MagicMock()
    tcp_mock = MagicMock()
    drone = Drone(on_telemetry=on_telemetry)
    drone._udp = udp_mock
    drone._tcp = tcp_mock
    drone._local_ip = "172.16.10.2"
    return drone, udp_mock, tcp_mock


def _identify_packet() -> bytes:
    return build(0x09, "SJ19F11-GPS".encode("gbk"))


def _inject_identify(drone: Drone) -> None:
    """Push a DroneInfo packet into the drone as if the UDP loop received it."""
    drone._handle_udp(_identify_packet())


# ---------------------------------------------------------------------------
# DroneState
# ---------------------------------------------------------------------------

class TestDroneState:
    def test_update_battery(self):
        s = DroneState()
        s.update(BatteryData(drone_raw=120, rc_raw=100))
        assert s.battery_drone_raw == 120
        assert s.battery_rc_raw == 100

    def test_update_gps(self):
        s = DroneState()
        s.update(GpsAttitude(longitude=-97.7, latitude=30.2, gps_signal=3, heading=90))
        assert s.longitude == -97.7
        assert s.latitude == 30.2
        assert s.heading == 90

    def test_update_flight(self):
        s = DroneState()
        s.update(FlightData(distance_m=10.0, altitude_m=5.0,
                            speed_horiz_ms=1.5, speed_vert_ms=0.0, status=0))
        assert s.altitude_m == 5.0
        assert s.distance_m == 10.0

    def test_update_model(self):
        s = DroneState()
        s.update(DroneInfo(raw="F11-GPS"))
        assert s.model == "F11-GPS"

    def test_snapshot_returns_dict(self):
        s = DroneState()
        s.update(BatteryData(drone_raw=80, rc_raw=70))
        snap = s.snapshot()
        assert isinstance(snap, dict)
        assert snap["battery_drone_raw"] == 80

    def test_snapshot_is_copy(self):
        s = DroneState()
        snap = s.snapshot()
        snap["altitude_m"] = 999
        assert s.altitude_m is None   # original unaffected


# ---------------------------------------------------------------------------
# connect() — startup sequence
# ---------------------------------------------------------------------------

class TestConnect:
    def test_identify_probe_sent_eight_times(self):
        drone, udp, tcp = _make_drone()

        # Inject identify response after a short delay
        def _respond():
            time.sleep(0.05)
            _inject_identify(drone)
        threading.Thread(target=_respond, daemon=True).start()

        with patch("process.drone.time.sleep"):  # skip 200ms waits
            drone.connect(timeout=2.0)

        identify_calls = [c for c in udp.send.call_args_list
                          if c.args[0] == cmd.IDENTIFY]
        assert len(identify_calls) == 8

    def test_heartbeat_thread_starts_after_identify(self):
        drone, udp, tcp = _make_drone()

        def _respond():
            time.sleep(0.05)
            _inject_identify(drone)
        threading.Thread(target=_respond, daemon=True).start()

        with patch("process.drone.time.sleep"):
            drone.connect(timeout=2.0)

        assert drone._hb_thread is not None
        assert drone._hb_thread.is_alive()
        drone.disconnect()

    def test_timeout_raises(self):
        drone, udp, tcp = _make_drone()
        with patch("process.drone.time.sleep"):
            with pytest.raises(TimeoutError):
                drone.connect(timeout=0.1)

    def test_model_set_after_connect(self):
        drone, udp, tcp = _make_drone()

        def _respond():
            time.sleep(0.05)
            _inject_identify(drone)
        threading.Thread(target=_respond, daemon=True).start()

        with patch("process.drone.time.sleep"):
            drone.connect(timeout=2.0)

        assert drone.state.model == "F11-GPS"
        drone.disconnect()


# ---------------------------------------------------------------------------
# Heartbeat thread
# ---------------------------------------------------------------------------

class TestHeartbeat:
    def test_sends_heartbeat_and_video_request(self):
        drone, udp, tcp = _make_drone()
        with patch("process.drone._HEARTBEAT_INTERVAL", 0.02):
            drone._start_heartbeat()
            time.sleep(0.08)   # ≥3 ticks at 20ms interval
            drone._stop_heartbeat()

        sent = [c.args[0] for c in udp.send.call_args_list]
        hb_pkts = [p for p in sent if p[3] == 0x09]
        vr_pkts = [p for p in sent if p[3] == 0x08]
        assert len(hb_pkts) >= 2
        assert len(vr_pkts) >= 2


# ---------------------------------------------------------------------------
# Control thread
# ---------------------------------------------------------------------------

class TestControlThread:
    def test_starts_and_sends_packets(self):
        drone, udp, tcp = _make_drone()
        with patch("process.drone._CONTROL_INTERVAL", 0.01):
            drone._start_control_thread()
            time.sleep(0.06)
            drone._stop_control_thread()

        ctrl_pkts = [c.args[0] for c in udp.send.call_args_list
                     if c.args[0][3] == 0x02]
        assert len(ctrl_pkts) >= 3

    def test_double_start_is_idempotent(self):
        drone, udp, tcp = _make_drone()
        drone._start_control_thread()
        t1 = drone._ctrl_thread
        drone._start_control_thread()
        assert drone._ctrl_thread is t1
        drone._stop_control_thread()

    def test_neutral_packets_when_no_flags(self):
        drone, udp, tcp = _make_drone()
        with patch("process.drone._CONTROL_INTERVAL", 0.01):
            drone._start_control_thread()
            time.sleep(0.05)
            drone._stop_control_thread()

        ctrl_pkts = [c.args[0] for c in udp.send.call_args_list
                     if c.args[0][3] == 0x02]
        for pkt in ctrl_pkts:
            assert pkt[4] == 0x00


# ---------------------------------------------------------------------------
# Flight commands
# ---------------------------------------------------------------------------

class TestFlightCommands:
    def test_takeoff_sets_flag(self):
        drone, udp, tcp = _make_drone()
        drone.takeoff()
        assert drone._flag_takeoff is True
        drone._stop_control_thread()

    def test_takeoff_starts_control_thread(self):
        drone, udp, tcp = _make_drone()
        drone.takeoff()
        assert drone._ctrl_thread is not None
        assert drone._ctrl_thread.is_alive()
        drone._stop_control_thread()

    def test_takeoff_flag_clears_after_hold(self):
        drone, udp, tcp = _make_drone()
        with patch("process.drone._FLAG_HOLD_S", 0.05):
            drone.takeoff()
            time.sleep(0.15)
        assert drone._flag_takeoff is False
        drone._stop_control_thread()

    def test_land_sets_flag(self):
        drone, udp, tcp = _make_drone()
        drone.land()
        assert drone._flag_land is True
        drone._stop_control_thread()

    def test_rth_sets_flag(self):
        drone, udp, tcp = _make_drone()
        drone.rth()
        assert drone._flag_rth is True
        drone._stop_control_thread()

    def test_cancel_rth_clears_flag(self):
        drone, udp, tcp = _make_drone()
        drone.rth()
        drone.cancel_rth()
        assert drone._flag_rth is False
        drone._stop_control_thread()

    def test_emergency_stop_sets_flag(self):
        drone, udp, tcp = _make_drone()
        drone.emergency_stop()
        assert drone._flag_stop is True
        drone._stop_control_thread()

    def test_takeoff_packet_has_takeoff_bit(self):
        drone, udp, tcp = _make_drone()
        drone._start_control_thread()
        with drone._ctrl_lock:
            drone._flag_takeoff = True
        time.sleep(0.05)
        with drone._ctrl_lock:
            drone._flag_takeoff = False
        drone._stop_control_thread()

        ctrl_pkts = [c.args[0] for c in udp.send.call_args_list
                     if c.args[0][3] == 0x02]
        takeoff_pkts = [p for p in ctrl_pkts if p[4] & 0x01]
        assert len(takeoff_pkts) >= 1


# ---------------------------------------------------------------------------
# Camera
# ---------------------------------------------------------------------------

class TestCamera:
    def test_capture_photo_delegates_to_tcp(self):
        drone, udp, tcp = _make_drone()
        tcp.capture_photo.return_value = b"\xFF\xD8\xFF\xE0"
        result = drone.capture_photo()
        assert result == b"\xFF\xD8\xFF\xE0"
        tcp.capture_photo.assert_called_once()

    def test_start_recording(self):
        drone, udp, tcp = _make_drone()
        drone.start_recording()
        tcp.send_raw.assert_called_once_with(bytes([0x12, 0x12]))

    def test_stop_recording(self):
        drone, udp, tcp = _make_drone()
        drone.stop_recording()
        tcp.send_raw.assert_called_once_with(bytes([0x13, 0x13]))


# ---------------------------------------------------------------------------
# Telemetry callback
# ---------------------------------------------------------------------------

class TestTelemetryCallback:
    def test_callback_called_with_parsed_packet(self):
        received = []
        drone, udp, tcp = _make_drone(on_telemetry=received.append)

        pkt = build(0x00, bytes([0x78, 0x64]))
        drone._handle_udp(pkt)

        assert len(received) == 1
        assert isinstance(received[0], BatteryData)

    def test_invalid_packet_not_forwarded(self):
        received = []
        drone, udp, tcp = _make_drone(on_telemetry=received.append)
        drone._handle_udp(bytes([0x00, 0x00, 0x00, 0x00]))
        assert received == []

    def test_callback_exception_does_not_raise(self):
        def bad(_): raise RuntimeError("boom")
        drone, udp, tcp = _make_drone(on_telemetry=bad)
        pkt = build(0x00, bytes([0x78, 0x64]))
        drone._handle_udp(pkt)   # must not propagate


# ---------------------------------------------------------------------------
# Context manager
# ---------------------------------------------------------------------------

class TestContextManager:
    def test_disconnect_called_on_exit(self):
        drone, udp, tcp = _make_drone()

        def _respond():
            time.sleep(0.05)
            _inject_identify(drone)
        threading.Thread(target=_respond, daemon=True).start()

        with patch("process.drone.time.sleep"):
            with drone:
                pass

        udp.stop.assert_called()
        tcp.disconnect.assert_called()