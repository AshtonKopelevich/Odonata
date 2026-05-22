"""
TCP socket wrapper for camera/system commands on port 8888.

Responsibilities:
  - Maintain a persistent TCP connection to the drone
  - Send raw command bytes
  - Receive variable-length responses (photo data uses a 6-byte header)
  - Reconnect automatically on connection loss

Response framing:
  Photo (0x1111): FF FF [4-byte big-endian size] [JPEG bytes]
  Most others:    raw bytes / short string (e.g. "stop\n", "no card\n")

Usage:
    tcp = TCPSocket()
    tcp.connect()
    tcp.send_raw(bytes([0x12, 0x12]))   # start recording
    jpeg = tcp.capture_photo()
    tcp.disconnect()
"""

import socket
import struct
import logging
from typing import Optional

log = logging.getLogger(__name__)

DRONE_IP    = "172.16.10.1"
TCP_PORT    = 8888
CONNECT_TIMEOUT_S = 5.0
RECV_TIMEOUT_S    = 5.0
PHOTO_MAGIC       = bytes([0xFF, 0xFF])
PHOTO_CMD         = bytes([0x11, 0x11, 0x00, 0x00])


class TCPSocket:
    def __init__(
        self,
        host: str = DRONE_IP,
        port: int = TCP_PORT,
    ) -> None:
        self._host = host
        self._port = port
        self._sock: Optional[socket.socket] = None

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def connect(self) -> None:
        """Open TCP connection to the drone."""
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._sock.settimeout(CONNECT_TIMEOUT_S)
        self._sock.connect((self._host, self._port))
        self._sock.settimeout(RECV_TIMEOUT_S)
        log.debug("TCP connected to %s:%d", self._host, self._port)

    def disconnect(self) -> None:
        """Close the TCP connection."""
        if self._sock:
            try:
                self._sock.close()
            except OSError:
                pass
            self._sock = None
        log.debug("TCP disconnected")

    @property
    def connected(self) -> bool:
        return self._sock is not None

    # ------------------------------------------------------------------
    # Send
    # ------------------------------------------------------------------

    def send_raw(self, data: bytes) -> None:
        """
        Send raw bytes over TCP.

        Raises:
            ConnectionError: if not connected or send fails.
        """
        if not self._sock:
            raise ConnectionError("TCP socket not connected")
        try:
            self._sock.sendall(data)
        except OSError as exc:
            self.disconnect()
            raise ConnectionError(f"TCP send failed: {exc}") from exc

    # ------------------------------------------------------------------
    # Receive helpers
    # ------------------------------------------------------------------

    def _recv_exact(self, n: int) -> bytes:
        """Read exactly n bytes, raising ConnectionError on short read."""
        buf = bytearray()
        while len(buf) < n:
            chunk = self._sock.recv(n - len(buf))
            if not chunk:
                raise ConnectionError("TCP connection closed by drone")
            buf.extend(chunk)
        return bytes(buf)

    def recv_response(self, max_bytes: int = 256) -> bytes:
        """
        Read a short response (non-photo commands).
        Returns whatever arrives up to max_bytes.
        """
        if not self._sock:
            raise ConnectionError("TCP socket not connected")
        try:
            return self._sock.recv(max_bytes)
        except TimeoutError:
            return b""
        except OSError as exc:
            self.disconnect()
            raise ConnectionError(f"TCP recv failed: {exc}") from exc

    # ------------------------------------------------------------------
    # Photo capture
    # ------------------------------------------------------------------

    def capture_photo(self) -> bytes:
        """
        Send photo command and receive the JPEG payload.

        Response format: FF FF [4-byte BE size] [JPEG bytes]

        Returns:
            Raw JPEG bytes.

        Raises:
            ConnectionError: on socket error.
            ValueError: if the response header is malformed.
        """
        self.send_raw(PHOTO_CMD)

        header = self._recv_exact(6)
        if header[:2] != PHOTO_MAGIC:
            raise ValueError(
                f"Bad photo response header: {header[:2].hex()}, expected ff ff"
            )
        size = struct.unpack(">I", header[2:6])[0]
        log.debug("Receiving JPEG: %d bytes", size)
        return self._recv_exact(size)

    # ------------------------------------------------------------------
    # Context manager support
    # ------------------------------------------------------------------

    def __enter__(self) -> "TCPSocket":
        self.connect()
        return self

    def __exit__(self, *_) -> None:
        self.disconnect()