"""
Outgoing packet factories (PC → Drone).

Each function returns a ready-to-send bytes object.
All encoding details are isolated here; callers never touch raw bytes.

Command reference (protocol.md §4):
  0x04  Identify probe
  0x08  Video stream request  (carry PC IP, sent every 1000ms)
  0x09  Heartbeat             (carry PC IP, sent every 1000ms)
  0x02  Control               (flight flags + rotate, sent every 20ms)
  0x01  Follow-me             (GPS position, deferred)
  0x07  Waypoint mission      (deferred)
"""

import socket
import struct
from process.protocol.packet import build

# ---------------------------------------------------------------------------
# IP encoding
# ---------------------------------------------------------------------------

def _encode_ip(ip_str: str) -> bytes:
    """
    Encode a dotted-decimal IP as a little-endian 32-bit integer (4 bytes).
    Matches Android WifiManager convention used by the original app.

    Example: "172.16.10.100" -> b'\\x64\\x0a\\x10\\xac'
    """
    return struct.pack("<I", struct.unpack("!I", socket.inet_aton(ip_str))[0])


# ---------------------------------------------------------------------------
# Command: 0x04  Identify probe
# ---------------------------------------------------------------------------

#: Pre-built identify packet (payload is always fixed).
IDENTIFY = build(0x04, bytes([0x01, 0x07]))


# ---------------------------------------------------------------------------
# Command: 0x09  Heartbeat
# ---------------------------------------------------------------------------

def heartbeat(ip: str) -> bytes:
    """
    Heartbeat packet telling the drone where to send telemetry.
    Sent every 1000 ms.

    Args:
        ip: Local IP address on the drone subnet (e.g. "172.16.10.2").
    """
    return build(0x09, _encode_ip(ip))


# ---------------------------------------------------------------------------
# Command: 0x08  Video stream request
# ---------------------------------------------------------------------------

def video_request(ip: str) -> bytes:
    """
    Video stream request telling the drone where to send the RTSP stream.
    Sent every 1000 ms.

    Args:
        ip: Local IP address on the drone subnet.
    """
    return build(0x08, _encode_ip(ip))


# ---------------------------------------------------------------------------
# Command: 0x02  Control packet
# ---------------------------------------------------------------------------

# Bit positions for the flags byte (byte 4 of control packet payload)
_FLAG_TAKEOFF  = 0x01   # bit 0
_FLAG_LAND     = 0x02   # bit 1
_FLAG_RTH      = 0x04   # bit 2
_FLAG_STOP     = 0x80   # bit 7

_DEFAULT_ROTATE = 64    # neutral yaw speed; sent as rotate*2 = 128


def control(
    takeoff: bool = False,
    land:    bool = False,
    rth:     bool = False,
    stop:    bool = False,
    rotate:  int  = _DEFAULT_ROTATE,
) -> bytes:
    """
    Build a control packet.  At most one of takeoff/land/rth/stop should be
    True at a time; the drone interprets the first set bit.

    Args:
        takeoff: Set takeoff flag (one-shot, clear after 1000 ms).
        land:    Set land flag (one-shot, clear after 1000 ms).
        rth:     Set return-to-home flag (hold until cancelled).
        stop:    Emergency stop.
        rotate:  Yaw speed 0-127; default 64.  Sent on wire as rotate*2.

    Returns:
        12-byte control packet.
    """
    flags = 0x00
    if takeoff: flags |= _FLAG_TAKEOFF
    if land:    flags |= _FLAG_LAND
    if rth:     flags |= _FLAG_RTH
    if stop:    flags |= _FLAG_STOP

    payload = bytes([
        flags,
        0x7F,           # b5 fixed
        0x7F,           # b6 fixed
        0x80,           # b7 fixed
        rotate * 2,     # rotate field
        0x20,           # b9 fixed
        0x20,           # b10 fixed
    ])
    return build(0x02, payload)


#: Neutral idle control packet — pre-built since it's sent at 50 Hz.
CONTROL_NEUTRAL = control()