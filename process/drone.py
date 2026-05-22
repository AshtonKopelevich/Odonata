"""
High-level Drone API.

This is the only module callers should import directly.  Everything below
(sockets, packet assembly, telemetry parsing) is an implementation detail.

Startup sequence (protocol.md §6):
  1. UDP + TCP sockets opened
  2. Identify probe sent ×8 at 200 ms intervals
  3. Wait for 0x09 DroneInfo response (confirms model)
  4. Heartbeat thread started  (every 1000 ms, cmd 0x09)
  5. Video-request thread started (every 1000 ms, cmd 0x08)
  6. Telemetry listener active (driven by UDP recv callback)

Control thread lifecycle (protocol.md §4.4):
  - Started on: takeoff / land / rth
  - Sends CONTROL_NEUTRAL at 50 Hz while running
  - One-shot flags (takeoff, land) are held for 1000 ms then cleared
  - Stopped 1000 ms after: land completes, rth cancelled

Thread inventory:
  udp-recv        UDPSocket internal — always running while connected
  hb-timer        heartbeat + video-request, 1 Hz
  ctrl-loop       50 Hz control packets, on-demand
"""

import socket
import threading
import time
import logging
from typing import Optional, Callable

from process.connection.udp import UDPSocket
from process.connection.tcp import TCPSocket
from process.protocol import commands as cmd
from process.protocol.telemetry import (
    parse_telemetry,
    BatteryData, GpsAttitude, FlightData,
    DroneInfo, SettingsStatus,
    TelemetryPacket,
)

log = logging.getLogger(__name__)

# Timing constants (seconds)
_IDENTIFY_COUNT    = 8
_IDENTIFY_INTERVAL = 0.2
_IDENTIFY_TIMEOUT  = 5.0
_HEARTBEAT_INTERVAL = 1.0
_CONTROL_INTERVAL   = 0.020   # 50 Hz
_FLAG_HOLD_S        = 1.0     # how long one-shot flags stay set
_CONTROL_LINGER_S   = 1.0     # how long control thread runs after land/rth-cancel


class DroneState:
    """Thread-safe snapshot of the latest telemetry from the drone."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self.battery_drone_raw: Optional[int] = None
        self.battery_rc_raw:    Optional[int] = None
        self.latitude:          Optional[float] = None
        self.longitude:         Optional[float] = None
        self.gps_signal:        Optional[int] = None
        self.heading:           Optional[int] = None
        self.altitude_m:        Optional[float] = None
        self.distance_m:        Optional[float] = None
        self.speed_horiz_ms:    Optional[float] = None
        self.speed_vert_ms:     Optional[float] = None
        self.model:             Optional[str] = None

    def update(self, packet: TelemetryPacket) -> None:
        with self._lock:
            if isinstance(packet, BatteryData):
                self.battery_drone_raw = packet.drone_raw
                self.battery_rc_raw    = packet.rc_raw
            elif isinstance(packet, GpsAttitude):
                self.longitude  = packet.longitude
                self.latitude   = packet.latitude
                self.gps_signal = packet.gps_signal
                self.heading    = packet.heading
            elif isinstance(packet, FlightData):
                self.altitude_m     = packet.altitude_m
                self.distance_m     = packet.distance_m
                self.speed_horiz_ms = packet.speed_horiz_ms
                self.speed_vert_ms  = packet.speed_vert_ms
            elif isinstance(packet, DroneInfo):
                self.model = packet.raw

    def snapshot(self) -> dict:
        """Return a plain dict copy — safe to read from any thread."""
        with self._lock:
            return {
                "battery_drone_raw": self.battery_drone_raw,
                "battery_rc_raw":    self.battery_rc_raw,
                "latitude":          self.latitude,
                "longitude":         self.longitude,
                "gps_signal":        self.gps_signal,
                "heading":           self.heading,
                "altitude_m":        self.altitude_m,
                "distance_m":        self.distance_m,
                "speed_horiz_ms":    self.speed_horiz_ms,
                "speed_vert_ms":     self.speed_vert_ms,
                "model":             self.model,
            }


def _local_ip(drone_ip: str = "172.16.10.1") -> str:
    """Discover the local IP on the drone's subnet without sending any traffic."""
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as s:
        s.connect((drone_ip, 80))
        return s.getsockname()[0]


class Drone:
    """
    High-level interface to the SJRC F11-GPS drone.

    Usage:
        drone = Drone()
        drone.connect()          # runs startup sequence, blocks until confirmed
        drone.takeoff()
        time.sleep(5)
        drone.land()
        drone.disconnect()

    Or as a context manager:
        with Drone() as drone:
            drone.takeoff()
            ...
    """

    def __init__(
        self,
        host: str = "172.16.10.1",
        on_telemetry: Optional[Callable[[TelemetryPacket], None]] = None,
    ) -> None:
        """
        Args:
            host:         Drone IP (default 172.16.10.1).
            on_telemetry: Optional callback invoked on every parsed telemetry
                          packet, called from the UDP recv thread.
        """
        self._host        = host
        self._on_telemetry = on_telemetry
        self._local_ip:   Optional[str] = None

        self._udp = UDPSocket(host=host, on_packet=self._handle_udp)
        self._tcp = TCPSocket(host=host)

        self.state = DroneState()

        # Startup synchronisation
        self._identified = threading.Event()

        # Background threads
        self._hb_thread:   Optional[threading.Thread] = None
        self._ctrl_thread: Optional[threading.Thread] = None
        self._stop_hb      = threading.Event()
        self._stop_ctrl    = threading.Event()

        # Control flags — written from caller thread, read from ctrl thread
        self._ctrl_lock   = threading.Lock()
        self._flag_takeoff = False
        self._flag_land    = False
        self._flag_rth     = False
        self._flag_stop    = False

    # ------------------------------------------------------------------
    # Connect / disconnect
    # ------------------------------------------------------------------

    def connect(self, timeout: float = _IDENTIFY_TIMEOUT) -> None:
        """
        Open sockets, run startup sequence, and wait for drone confirmation.

        Args:
            timeout: Seconds to wait for the identify response.

        Raises:
            TimeoutError: if the drone does not respond within timeout.
            ConnectionError: on socket errors.
        """
        self._local_ip = _local_ip(self._host)
        log.info("Local IP on drone subnet: %s", self._local_ip)

        self._udp.start()
        self._tcp.connect()

        # Send identify probe ×8 at 200 ms
        for _ in range(_IDENTIFY_COUNT):
            self._udp.send(cmd.IDENTIFY)
            time.sleep(_IDENTIFY_INTERVAL)

        if not self._identified.wait(timeout=timeout):
            self.disconnect()
            raise TimeoutError("Drone did not respond to identify probe")

        log.info("Drone identified: %s", self.state.model)

        self._start_heartbeat()
        log.info("Connected and ready")

    def disconnect(self) -> None:
        """Stop all threads and close sockets."""
        self._stop_control_thread()
        self._stop_heartbeat()
        self._udp.stop()
        self._tcp.disconnect()
        log.info("Disconnected")

    # ------------------------------------------------------------------
    # Flight commands
    # ------------------------------------------------------------------

    def takeoff(self) -> None:
        """
        Command the drone to take off.
        Starts the control thread; takeoff flag is held for 1000 ms then cleared.
        """
        log.info("Takeoff")
        self._start_control_thread()
        self._set_flag("takeoff")

    def land(self) -> None:
        """
        Command the drone to land.
        Land flag held for 1000 ms, then control thread stops after a further 1000 ms.
        """
        log.info("Land")
        self._start_control_thread()
        self._set_flag("land", stop_after=True)

    def rth(self) -> None:
        """Return to home. Control thread runs until cancel_rth() is called."""
        log.info("RTH")
        self._start_control_thread()
        with self._ctrl_lock:
            self._flag_rth = True

    def cancel_rth(self) -> None:
        """Cancel return-to-home; stops control thread after linger."""
        log.info("RTH cancelled")
        with self._ctrl_lock:
            self._flag_rth = False
        self._schedule_control_stop()

    def emergency_stop(self) -> None:
        """Set emergency stop flag immediately."""
        log.warning("EMERGENCY STOP")
        with self._ctrl_lock:
            self._flag_stop = True
        self._start_control_thread()

    # ------------------------------------------------------------------
    # Camera
    # ------------------------------------------------------------------

    def capture_photo(self) -> bytes:
        """
        Capture a photo and return the raw JPEG bytes.

        Raises:
            ConnectionError: if TCP is not connected.
            ValueError: if the drone response is malformed.
        """
        log.info("Capturing photo")
        return self._tcp.capture_photo()

    def start_recording(self) -> None:
        self._tcp.send_raw(bytes([0x12, 0x12]))

    def stop_recording(self) -> None:
        self._tcp.send_raw(bytes([0x13, 0x13]))

    # ------------------------------------------------------------------
    # Telemetry
    # ------------------------------------------------------------------

    def get_state(self) -> dict:
        """Return a plain-dict snapshot of the latest telemetry."""
        return self.state.snapshot()

    # ------------------------------------------------------------------
    # Internal: UDP receive callback
    # ------------------------------------------------------------------

    def _handle_udp(self, data: bytes) -> None:
        packet = parse_telemetry(data)
        if packet is None:
            return

        # Signal startup completion on first DroneInfo
        if isinstance(packet, DroneInfo):
            self._identified.set()

        # Handle settings resend request
        if isinstance(packet, SettingsStatus) and packet.code == 0x01:
            log.debug("Drone requested settings resend (ignored in base impl)")

        self.state.update(packet)

        if self._on_telemetry:
            try:
                self._on_telemetry(packet)
            except Exception:
                log.exception("Exception in on_telemetry callback")

    # ------------------------------------------------------------------
    # Internal: heartbeat thread
    # ------------------------------------------------------------------

    def _start_heartbeat(self) -> None:
        self._stop_hb.clear()
        self._hb_thread = threading.Thread(
            target=self._hb_loop,
            name="hb-timer",
            daemon=True,
        )
        self._hb_thread.start()

    def _stop_heartbeat(self) -> None:
        self._stop_hb.set()
        if self._hb_thread:
            self._hb_thread.join(timeout=3.0)

    def _hb_loop(self) -> None:
        while True:
            self._udp.send(cmd.heartbeat(self._local_ip))
            self._udp.send(cmd.video_request(self._local_ip))
            if self._stop_hb.wait(timeout=_HEARTBEAT_INTERVAL):
                break

    # ------------------------------------------------------------------
    # Internal: control thread
    # ------------------------------------------------------------------

    def _start_control_thread(self) -> None:
        if self._ctrl_thread and self._ctrl_thread.is_alive():
            return
        self._stop_ctrl.clear()
        self._ctrl_thread = threading.Thread(
            target=self._ctrl_loop,
            name="ctrl-loop",
            daemon=True,
        )
        self._ctrl_thread.start()

    def _stop_control_thread(self) -> None:
        self._stop_ctrl.set()
        if self._ctrl_thread:
            self._ctrl_thread.join(timeout=3.0)

    def _schedule_control_stop(self) -> None:
        """Stop the control thread after the linger delay (non-blocking)."""
        def _delayed():
            time.sleep(_CONTROL_LINGER_S)
            self._stop_ctrl.set()
        threading.Thread(target=_delayed, daemon=True).start()

    def _set_flag(self, flag: str, stop_after: bool = False) -> None:
        """
        Set a one-shot control flag, then clear it after FLAG_HOLD_S.
        If stop_after is True, also schedule the control thread to stop.
        """
        with self._ctrl_lock:
            setattr(self, f"_flag_{flag}", True)

        def _clear():
            time.sleep(_FLAG_HOLD_S)
            with self._ctrl_lock:
                setattr(self, f"_flag_{flag}", False)
            if stop_after:
                self._schedule_control_stop()

        threading.Thread(target=_clear, daemon=True).start()

    def _ctrl_loop(self) -> None:
        """Send control packets at 50 Hz until stop is signalled."""
        while not self._stop_ctrl.is_set():
            with self._ctrl_lock:
                pkt = cmd.control(
                    takeoff=self._flag_takeoff,
                    land=self._flag_land,
                    rth=self._flag_rth,
                    stop=self._flag_stop,
                )
            self._udp.send(pkt)
            time.sleep(_CONTROL_INTERVAL)

    # ------------------------------------------------------------------
    # Context manager
    # ------------------------------------------------------------------

    def __enter__(self) -> "Drone":
        self.connect()
        return self

    def __exit__(self, *_) -> None:
        self.disconnect()