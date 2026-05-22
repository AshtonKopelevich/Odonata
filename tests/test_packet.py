"""
Tests for protocol/packet.py.

All expected byte sequences are taken directly from protocol.md
so failures pinpoint protocol regressions, not test errors.
"""

import sys
import os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest
from process.protocol.packet import build, validate, parse, checksum


# ---------------------------------------------------------------------------
# Checksum
# ---------------------------------------------------------------------------

class TestChecksum:
    # NOTE: protocol.md §3.2 says "XOR of bytes 2 through (last-1)" which would
    # include the CMD byte.  However, the doc's own example packets prove otherwise:
    #   identify probe:  0x03^0x01^0x07 = 0x05  (skips CMD=0x04)
    #   neutral control: 0x08^0x00^0x7F^0x7F^0x80^0x80^0x20^0x20 = 0x08  (skips CMD=0x02)
    # Actual formula: len_byte XOR payload_bytes (CMD byte excluded).

    def test_identify_probe(self):
        # 5A 55 03 04 01 07 05  — doc-provided full packet, checksum = 0x05
        buf = bytearray([0x5A, 0x55, 0x03, 0x04, 0x01, 0x07, 0x00])
        assert checksum(buf) == 0x05

    def test_neutral_control(self):
        # 5A 55 08 02 00 7F 7F 80 80 20 20 08  — doc-provided, checksum = 0x08
        buf = bytearray([0x5A, 0x55, 0x08, 0x02, 0x00,
                         0x7F, 0x7F, 0x80, 0x80, 0x20, 0x20, 0x00])
        assert checksum(buf) == 0x08


# ---------------------------------------------------------------------------
# build()
# ---------------------------------------------------------------------------

class TestBuild:
    def test_identify_probe(self):
        # From protocol.md §4.1: 5A 55 03 04 01 07 05
        pkt = build(0x04, bytes([0x01, 0x07]))
        assert pkt == bytes([0x5A, 0x55, 0x03, 0x04, 0x01, 0x07, 0x05])

    def test_neutral_control_packet(self):
        # From protocol.md §4.4: 5A 55 08 02 00 7F 7F 80 80 20 20 08
        payload = bytes([0x00, 0x7F, 0x7F, 0x80, 0x80, 0x20, 0x20])
        pkt = build(0x02, payload)
        assert pkt == bytes([0x5A, 0x55, 0x08, 0x02, 0x00,
                              0x7F, 0x7F, 0x80, 0x80, 0x20, 0x20, 0x08])

    def test_length_field(self):
        # N = 1 (cmd) + len(payload); total = N + 4
        pkt = build(0x09, bytes([0x01, 0x02, 0x03, 0x04]))
        assert pkt[2] == 5       # N = 1 cmd + 4 payload
        assert len(pkt) == 9     # 5 + 4

    def test_no_payload(self):
        pkt = build(0xFF)
        assert len(pkt) == 4 + 1  # magic(2) + len(1) + cmd(1) + checksum(1)
        assert pkt[2] == 1

    def test_checksum_is_last_byte(self):
        pkt = build(0x09, bytes([0xAA, 0xBB, 0xCC, 0xDD]))
        assert pkt[-1] == checksum(pkt)


# ---------------------------------------------------------------------------
# validate()
# ---------------------------------------------------------------------------

class TestValidate:
    def test_valid_known_packet(self):
        pkt = bytes([0x5A, 0x55, 0x03, 0x04, 0x01, 0x07, 0x05])
        assert validate(pkt) is True

    def test_bad_magic_1(self):
        pkt = bytes([0x00, 0x55, 0x03, 0x04, 0x01, 0x07, 0x05])
        assert validate(pkt) is False

    def test_bad_magic_2(self):
        pkt = bytes([0x5A, 0x00, 0x03, 0x04, 0x01, 0x07, 0x05])
        assert validate(pkt) is False

    def test_bad_checksum(self):
        pkt = bytearray([0x5A, 0x55, 0x03, 0x04, 0x01, 0x07, 0xFF])
        assert validate(pkt) is False

    def test_too_short(self):
        assert validate(bytes([0x5A, 0x55, 0x01])) is False

    def test_truncated_payload(self):
        # Length byte says 3 but only 2 bytes of payload provided
        pkt = bytes([0x5A, 0x55, 0x03, 0x04, 0x01])
        assert validate(pkt) is False

    def test_roundtrip(self):
        """build() then validate() must always pass."""
        for cmd in [0x01, 0x02, 0x04, 0x08, 0x09]:
            pkt = build(cmd, bytes([0x01, 0x02, 0x03, 0x04]))
            assert validate(pkt), f"Roundtrip failed for cmd={cmd:#x}"


# ---------------------------------------------------------------------------
# parse()
# ---------------------------------------------------------------------------

class TestParse:
    def test_identify_probe(self):
        pkt = bytes([0x5A, 0x55, 0x03, 0x04, 0x01, 0x07, 0x05])
        result = parse(pkt)
        assert result is not None
        cmd, payload = result
        assert cmd == 0x04
        assert payload == bytes([0x01, 0x07])

    def test_no_payload_cmd(self):
        pkt = build(0xFF)
        cmd, payload = parse(pkt)
        assert cmd == 0xFF
        assert payload == b""

    def test_invalid_returns_none(self):
        assert parse(bytes([0x00, 0x00, 0x00, 0x00])) is None

    def test_roundtrip_payload_integrity(self):
        original = bytes(range(16))
        pkt = build(0x07, original)
        cmd, payload = parse(pkt)
        assert cmd == 0x07
        assert payload == original