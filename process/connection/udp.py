"""
UDP socket wrapper for drone communication on port 8080.

Responsibilities:
  - Send arbitrary packets (fire-and-forget)
  - Receive incoming datagrams and dispatch to a callback
  - Run the receive loop in a background daemon thread

Threading model:
  - One background thread: _recv_loop (always running while connected)
  - Callers send from their own thread; UDP sendto is thread-safe on all
    major platforms for datagrams under MTU size.

Usage:
    def on_packet(data: bytes) -> None:
        ...

    sock = UDPSocket("172.16.10.1", on_packet=on_packet)
    sock.start()
    sock.send(heartbeat("172.16.10.2"))
    ...
    sock.stop()
"""

import socket
import threading
import logging
from typing import Callable, Optional

log = logging.getLogger(__name__)

DRONE_IP   = "172.16.10.1"
UDP_PORT   = 8080
RECV_BUF   = 4096
RECV_TIMEOUT_S = 1.0   # unblocks recv loop so stop() is responsive


class UDPSocket:
    def __init__(
        self,
        host: str = DRONE_IP,
        port: int = UDP_PORT,
        on_packet: Optional[Callable[[bytes], None]] = None,
    ) -> None:
        self._host = host
        self._port = port
        self._on_packet = on_packet
        self._sock: Optional[socket.socket] = None
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()

    # ------------------------------------------------------------------
    # Lifecycle
    # ------------------------------------------------------------------

    def start(self) -> None:
        """Open the socket and start the receive loop thread."""
        if self._thread and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sock.settimeout(RECV_TIMEOUT_S)
        # Bind to any local port so we can receive responses
        self._sock.bind(("", self._port))
        self._thread = threading.Thread(
            target=self._recv_loop,
            name="udp-recv",
            daemon=True,
        )
        self._thread.start()
        log.debug("UDP socket started (%s:%d)", self._host, self._port)

    def stop(self) -> None:
        """Signal the receive loop to exit and close the socket."""
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=3.0)
        if self._sock:
            self._sock.close()
            self._sock = None
        log.debug("UDP socket stopped")

    # ------------------------------------------------------------------
    # Send
    # ------------------------------------------------------------------

    def send(self, data: bytes) -> None:
        """
        Send a datagram to the drone.  Thread-safe.

        Silently drops the packet if the socket is not open.
        """
        if self._sock is None:
            log.warning("UDP send called before start()")
            return
        try:
            self._sock.sendto(data, (self._host, self._port))
        except OSError as exc:
            log.error("UDP send error: %s", exc)

    # ------------------------------------------------------------------
    # Receive loop (background thread)
    # ------------------------------------------------------------------

    def _recv_loop(self) -> None:
        while not self._stop_event.is_set():
            try:
                data, _ = self._sock.recvfrom(RECV_BUF)
            except TimeoutError:
                continue
            except OSError as exc:
                if not self._stop_event.is_set():
                    log.error("UDP recv error: %s", exc)
                break

            if self._on_packet:
                try:
                    self._on_packet(data)
                except Exception:
                    log.exception("Exception in UDP on_packet callback")

    # ------------------------------------------------------------------
    # Context manager support
    # ------------------------------------------------------------------

    def __enter__(self) -> "UDPSocket":
        self.start()
        return self

    def __exit__(self, *_) -> None:
        self.stop()