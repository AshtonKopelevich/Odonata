"""
Incoming packet parser (Drone → PC).

parse_packet() takes a raw validated payload (after packet.parse()) and
returns a typed dataclass, or None for unknown/unhandled commands.

Telemetry reference (protocol.md §5):
  0x00  Battery voltages
  0x01  GPS + attitude
  0x02  Distance / altitude / speed
  0x03  Photo notification
  0x04  Video notification
  0x05  Flight alert (string, GBK encoded)
  0x06  Close alert
  0x07  Follow-me status
  0x08  Waypoint status
  0x09  Drone info response
  0x0B  Settings status
  0x0C  Drone status
"""

from __future__ import annotations
import struct
from dataclasses import dataclass, field
from typing import Optional
from process.protocol.packet import parse as _parse_raw


# ---------------------------------------------------------------------------
# Telemetry dataclasses
# ---------------------------------------------------------------------------

@dataclass
class BatteryData:
    """Command 0x00 — raw voltage bytes from drone and RC."""
    drone_raw: int      # raw byte; see ElectricityUtils for conversion
    rc_raw:    int


@dataclass
class GpsAttitude:
    """Command 0x01 — GPS coordinates, signal, heading, optional tilt."""
    longitude:  float
    latitude:   float
    gps_signal: int             # 0-3
    heading:    int             # degrees
    tilt_fore_aft:   Optional[int] = None
    tilt_left_right: Optional[int] = None


@dataclass
class FlightData:
    """Command 0x02 — distance, altitude, speed."""
    distance_m:     float   # from home point
    altitude_m:     float
    speed_horiz_ms: float
    speed_vert_ms:  float
    status:         int     # 0x0F = invalid


@dataclass
class FollowMeStatus:
    """Command 0x07."""
    active: bool    # True = following, False = cancelled


@dataclass
class WaypointStatus:
    """Command 0x08."""
    code: int
    # 0x01 data received, 0x02 flying, 0x03 interrupted, 0x04 done, 0x05 unsuitable


@dataclass
class DroneInfo:
    """Command 0x09 — firmware/model string."""
    raw: str        # GBK-decoded, 'SJ19' prefix stripped by caller


@dataclass
class SettingsStatus:
    """Command 0x0B."""
    code: int
    # 0x00 ok, 0x01 resend, 0x02 normal stop, 0x03 took off, 0x08 e-stop, 0x0F low batt


@dataclass
class AlertData:
    """Command 0x05 — flight alert string."""
    message: str


@dataclass
class DroneStatus:
    """Command 0x0C — drone status code 0-5, or -1 = dismiss."""
    code: int


# Sentinel types for zero-payload notifications
@dataclass
class PhotoNotification:
    """Command 0x03."""

@dataclass
class VideoNotification:
    """Command 0x04."""

@dataclass
class CloseAlert:
    """Command 0x06."""


# Union of all possible parsed results
TelemetryPacket = (
    BatteryData | GpsAttitude | FlightData |
    FollowMeStatus | WaypointStatus | DroneInfo |
    SettingsStatus | AlertData | DroneStatus |
    PhotoNotification | VideoNotification | CloseAlert
)


# ---------------------------------------------------------------------------
# Voltage decoding
# ---------------------------------------------------------------------------

# The app uses ElectricityUtils.toFloat() which is not fully documented.
# From context: > 9.0V = 3S battery (F11).  Most likely scale is raw / 10.0.
def decode_voltage(raw: int) -> float:
    return raw / 10.0


# ---------------------------------------------------------------------------
# Per-command parsers (internal)
# ---------------------------------------------------------------------------

def _parse_battery(payload: bytes) -> BatteryData:
    return BatteryData(drone_raw=payload[0], rc_raw=payload[1])


def _parse_gps(payload: bytes) -> GpsAttitude:
    lon = struct.unpack_from("<i", payload, 0)[0] / 1e7
    lat = struct.unpack_from("<i", payload, 4)[0] / 1e7
    signal = payload[8]
    heading = (payload[9] << 8) | payload[10]
    tilt_fa = payload[11] if len(payload) > 12 else None
    tilt_lr = payload[12] if len(payload) > 12 else None
    return GpsAttitude(lon, lat, signal, heading, tilt_fa, tilt_lr)


_INVALID_STATUS = 0x0F
_INVALID_STREAK_THRESHOLD = 3

def _parse_flight(payload: bytes) -> FlightData:
    dist  = struct.unpack_from(">h", payload, 0)[0] / 10.0
    alt   = struct.unpack_from(">h", payload, 2)[0] / 10.0
    hspd  = struct.unpack_from(">h", payload, 4)[0] / 10.0
    vspd  = struct.unpack_from(">h", payload, 6)[0] / 10.0
    status = payload[8]
    # Clamp negatives to 0 per protocol note
    dist  = max(dist,  0.0)
    alt   = max(alt,   0.0)
    hspd  = max(hspd,  0.0)
    return FlightData(dist, alt, hspd, vspd, status)


def _parse_alert(payload: bytes) -> AlertData:
    try:
        msg = payload.decode("gbk")
    except UnicodeDecodeError:
        msg = payload.decode("latin-1")
    return AlertData(msg)


def _parse_drone_info(payload: bytes) -> DroneInfo:
    try:
        raw = payload.decode("gbk")
    except UnicodeDecodeError:
        raw = payload.decode("latin-1")
    if raw.startswith("SJ19"):
        raw = raw[4:]
    return DroneInfo(raw)


# ---------------------------------------------------------------------------
# Public dispatcher
# ---------------------------------------------------------------------------

_PARSERS = {
    0x00: lambda p: _parse_battery(p),
    0x01: lambda p: _parse_gps(p),
    0x02: lambda p: _parse_flight(p),
    0x03: lambda p: PhotoNotification(),
    0x04: lambda p: VideoNotification(),
    0x05: lambda p: _parse_alert(p),
    0x06: lambda p: CloseAlert(),
    0x07: lambda p: FollowMeStatus(active=(p[0] == 0x01)),
    0x08: lambda p: WaypointStatus(code=p[0]),
    0x09: lambda p: _parse_drone_info(p),
    0x0B: lambda p: SettingsStatus(code=p[0]),
    0x0C: lambda p: DroneStatus(code=p[0] if p[0] != 0xFF else -1),
}


def parse_telemetry(raw_buf: bytes) -> Optional[TelemetryPacket]:
    """
    Parse a raw UDP buffer into a typed telemetry object.

    Args:
        raw_buf: Complete UDP datagram including magic bytes.

    Returns:
        A typed dataclass, or None if the packet is invalid or unrecognised.
    """
    result = _parse_raw(raw_buf)
    if result is None:
        return None
    cmd, payload = result
    parser = _PARSERS.get(cmd)
    if parser is None:
        return None
    try:
        return parser(payload)
    except (IndexError, struct.error):
        return None