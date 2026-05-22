"""
Tests for connection/udp.py and connection/tcp.py.

All socket I/O is mocked — no network required.
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import struct
import threading
import time
from unittest.mock import MagicMock, patch, call
import pytest

from process.connection.udp import UDPSocket
from process.connection.tcp import TCPSocket, PHOTO_CMD, PHOTO_MAGIC


# ===========================================================================
# UDPSocket
# ===========================================================================

class TestUDPSocketLifecycle:
    def test_start_creates_socket_and_thread(self):
        with patch("process.connection.udp.socket.socket") as mock_sock_cls:
            mock_sock = MagicMock()
            mock_sock_cls.return_value = mock_sock
            # recvfrom blocks forever unless we stop it
            mock_sock.recvfrom.side_effect = TimeoutError

            udp = UDPSocket()
            udp.start()
            assert udp._thread is not None
            assert udp._thread.is_alive()
            udp.stop()

    def test_stop_joins_thread(self):
        with patch("process.connection.udp.socket.socket") as mock_sock_cls:
            mock_sock = MagicMock()
            mock_sock_cls.return_value = mock_sock
            mock_sock.recvfrom.side_effect = TimeoutError

            udp = UDPSocket()
            udp.start()
            udp.stop()
            assert not udp._thread.is_alive()

    def test_double_start_is_idempotent(self):
        with patch("process.connection.udp.socket.socket") as mock_sock_cls:
            mock_sock = MagicMock()
            mock_sock_cls.return_value = mock_sock
            mock_sock.recvfrom.side_effect = TimeoutError

            udp = UDPSocket()
            udp.start()
            thread_id = id(udp._thread)
            udp.start()   # second call should no-op
            assert id(udp._thread) == thread_id
            udp.stop()

    def test_context_manager(self):
        with patch("process.connection.udp.socket.socket") as mock_sock_cls:
            mock_sock = MagicMock()
            mock_sock_cls.return_value = mock_sock
            mock_sock.recvfrom.side_effect = TimeoutError

            with UDPSocket() as udp:
                assert udp._sock is not None
            assert udp._sock is None


class TestUDPSocketSend:
    def _make_started(self):
        patcher = patch("process.connection.udp.socket.socket")
        mock_sock_cls = patcher.start()
        mock_sock = MagicMock()
        mock_sock_cls.return_value = mock_sock
        mock_sock.recvfrom.side_effect = TimeoutError
        udp = UDPSocket(host="172.16.10.1", port=8080)
        udp.start()
        return udp, mock_sock, patcher

    def test_send_calls_sendto(self):
        udp, mock_sock, patcher = self._make_started()
        try:
            data = bytes([0x5A, 0x55, 0x01, 0x09, 0x09])
            udp.send(data)
            mock_sock.sendto.assert_called_once_with(data, ("172.16.10.1", 8080))
        finally:
            udp.stop()
            patcher.stop()

    def test_send_before_start_does_not_raise(self):
        udp = UDPSocket()
        udp.send(b"\x00")   # should log warning but not raise


class TestUDPSocketReceive:
    def _make_recvfrom(self, sequence):
        """
        Return a side_effect callable that yields items from sequence then
        raises TimeoutError indefinitely, avoiding StopIteration in the
        background thread when the mock's list runs dry.
        """
        it = iter(sequence)
        def _side_effect(*_):
            try:
                item = next(it)
                if isinstance(item, type) and issubclass(item, Exception):
                    raise item()
                if isinstance(item, Exception):
                    raise item
                return item
            except StopIteration:
                raise TimeoutError()
        return _side_effect

    def test_callback_called_on_packet(self):
        received = []

        with patch("process.connection.udp.socket.socket") as mock_sock_cls:
            mock_sock = MagicMock()
            mock_sock_cls.return_value = mock_sock

            packet = bytes([0x5A, 0x55, 0x01, 0x00, 0x00])
            mock_sock.recvfrom.side_effect = self._make_recvfrom([
                (packet, ("172.16.10.1", 8080)),
            ])

            udp = UDPSocket(on_packet=received.append)
            udp.start()
            time.sleep(0.05)
            udp.stop()

        assert received == [packet]

    def test_callback_exception_does_not_crash_loop(self):
        """A bad callback must not kill the receive thread."""
        call_count = [0]

        def bad_callback(data):
            call_count[0] += 1
            raise RuntimeError("boom")

        with patch("process.connection.udp.socket.socket") as mock_sock_cls:
            mock_sock = MagicMock()
            mock_sock_cls.return_value = mock_sock
            mock_sock.recvfrom.side_effect = self._make_recvfrom([
                (b"\x5A\x55\x01\x00\x00", ("172.16.10.1", 8080)),
                (b"\x5A\x55\x01\x00\x00", ("172.16.10.1", 8080)),
            ])

            udp = UDPSocket(on_packet=bad_callback)
            udp.start()
            time.sleep(0.05)
            udp.stop()

        assert call_count[0] == 2   # both packets attempted despite exceptions


# ===========================================================================
# TCPSocket
# ===========================================================================

class TestTCPSocketLifecycle:
    def test_connect_and_disconnect(self):
        with patch("process.connection.tcp.socket.socket") as mock_sock_cls:
            mock_sock = MagicMock()
            mock_sock_cls.return_value = mock_sock

            tcp = TCPSocket()
            assert not tcp.connected
            tcp.connect()
            assert tcp.connected
            mock_sock.connect.assert_called_once_with(("172.16.10.1", 8888))
            tcp.disconnect()
            assert not tcp.connected

    def test_context_manager(self):
        with patch("process.connection.tcp.socket.socket") as mock_sock_cls:
            mock_sock = MagicMock()
            mock_sock_cls.return_value = mock_sock

            with TCPSocket() as tcp:
                assert tcp.connected
            assert not tcp.connected


class TestTCPSocketSend:
    def test_send_raw(self):
        with patch("process.connection.tcp.socket.socket") as mock_sock_cls:
            mock_sock = MagicMock()
            mock_sock_cls.return_value = mock_sock

            with TCPSocket() as tcp:
                tcp.send_raw(bytes([0x12, 0x12]))
                mock_sock.sendall.assert_called_once_with(bytes([0x12, 0x12]))

    def test_send_raises_when_not_connected(self):
        tcp = TCPSocket()
        with pytest.raises(ConnectionError):
            tcp.send_raw(b"\x00")


class TestTCPPhotoCapture:
    def _make_photo_response(self, jpeg: bytes) -> list[bytes]:
        """Build the mock recv sequence for a photo response."""
        size = len(jpeg)
        header = PHOTO_MAGIC + struct.pack(">I", size)
        # _recv_exact calls recv in a loop; return header then jpeg in one chunk each
        return [header, jpeg]

    def test_capture_returns_jpeg(self):
        jpeg = b"\xFF\xD8\xFF\xE0" + b"\x00" * 100   # fake JPEG

        with patch("process.connection.tcp.socket.socket") as mock_sock_cls:
            mock_sock = MagicMock()
            mock_sock_cls.return_value = mock_sock
            mock_sock.recv.side_effect = self._make_photo_response(jpeg)

            with TCPSocket() as tcp:
                result = tcp.capture_photo()

        assert result == jpeg
        mock_sock.sendall.assert_called_once_with(PHOTO_CMD)

    def test_capture_bad_header_raises(self):
        with patch("process.connection.tcp.socket.socket") as mock_sock_cls:
            mock_sock = MagicMock()
            mock_sock_cls.return_value = mock_sock
            # Wrong magic bytes in header
            mock_sock.recv.return_value = bytes([0x00, 0x00, 0x00, 0x00, 0x00, 0x10])

            with TCPSocket() as tcp:
                with pytest.raises(ValueError, match="Bad photo response header"):
                    tcp.capture_photo()

    def test_recv_exact_handles_chunked_data(self):
        """_recv_exact must reassemble data arriving in multiple chunks."""
        jpeg = bytes(range(256))
        size = len(jpeg)
        header = PHOTO_MAGIC + struct.pack(">I", size)

        # Simulate header arriving in two chunks, then jpeg in two chunks
        half = size // 2
        chunks = [header[:3], header[3:], jpeg[:half], jpeg[half:]]

        with patch("process.connection.tcp.socket.socket") as mock_sock_cls:
            mock_sock = MagicMock()
            mock_sock_cls.return_value = mock_sock
            mock_sock.recv.side_effect = chunks

            with TCPSocket() as tcp:
                result = tcp.capture_photo()

        assert result == jpeg