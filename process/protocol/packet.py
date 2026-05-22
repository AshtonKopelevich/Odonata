"""
Low-level packet assembly and validation for the SJRC F11 UDP protocol.

Packet structure:
  Byte 0:    0x5A  (magic 1)
  Byte 1:    0x55  (magic 2)
  Byte 2:    N     (payload length; total packet = N + 4)
  Byte 3:    CMD   (command type)
  Byte 4...: PAYLOAD (N - 1 bytes, not including checksum byte)
  Last byte: XOR checksum of bytes 2..(last-1)

Note: payload length N counts the CMD byte plus all payload bytes,
but NOT the two magic bytes and NOT the checksum byte.
So: N = 1 (cmd) + len(payload_bytes)
    total packet len = N + 3 (magic×2 + length byte) + 1 (checksum) = N + 4
"""

MAGIC = bytes([0x5A, 0x55])


def checksum(buf: bytes | bytearray) -> int:
    """
    XOR of the length byte and all payload bytes, skipping the CMD byte.

    Despite the protocol doc stating 'bytes 2 through last-1', empirical
    verification of the doc's own example packets shows the CMD byte (byte 3)
    is excluded.  The actual range is: byte[2] XOR byte[4] XOR ... XOR byte[-2].
    """
    result = buf[2]          # length byte
    for b in buf[4:-1]:      # payload bytes (skips magic, length, cmd, checksum)
        result ^= b
    return result & 0xFF


def build(cmd: int, payload: bytes = b"") -> bytes:
    """
    Assemble a complete packet ready to send.

    Args:
        cmd:     Command byte (e.g. 0x09 for heartbeat).
        payload: Raw payload bytes (everything after the cmd byte).

    Returns:
        Complete packet as bytes, including magic, length, cmd, payload, checksum.
    """
    n = 1 + len(payload)          # cmd byte + payload bytes
    buf = bytearray(n + 4)        # magic(2) + len(1) + cmd(1) + payload + checksum(1)
    buf[0] = 0x5A
    buf[1] = 0x55
    buf[2] = n
    buf[3] = cmd
    buf[4:4 + len(payload)] = payload
    buf[-1] = checksum(buf)
    return bytes(buf)


def validate(buf: bytes | bytearray) -> bool:
    """
    Check magic bytes and XOR checksum of an incoming packet.

    Returns True if the packet is structurally valid.
    """
    if len(buf) < 4:
        return False
    if buf[0] != 0x5A or buf[1] != 0x55:
        return False
    expected_len = buf[2] + 4    # N + 4
    if len(buf) < expected_len:
        return False
    return buf[expected_len - 1] == checksum(buf[:expected_len])


def parse(buf: bytes | bytearray) -> tuple[int, bytes] | None:
    """
    Parse a validated incoming packet.

    Returns:
        (cmd, payload) if valid, None otherwise.
        payload does NOT include the cmd byte or checksum.
    """
    if not validate(buf):
        return None
    cmd = buf[3]
    payload_end = buf[2] + 3      # offset of checksum byte = 2 + N + 1
    payload = bytes(buf[4:payload_end])
    return cmd, payload