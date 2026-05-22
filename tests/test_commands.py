"""
Tests for protocol/commands.py.
No network required — all assertions are against known byte sequences.
"""

import sys, os
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

import pytest
from process.protocol.packet import validate
from process.protocol.commands import (
    IDENTIFY, CONTROL_NEUTRAL, heartbeat, video_request, control,
)


class TestIdentify:
    def test_known_bytes(self):
        # protocol.md §4.1: 5A 55 03 04 01 07 05
        assert IDENTIFY == bytes([0x5A, 0x55, 0x03, 0x04, 0x01, 0x07, 0x05])

    def test_is_valid_packet(self):
        assert validate(IDENTIFY)


class TestHeartbeat:
    def test_structure(self):
        pkt = heartbeat("172.16.10.2")
        assert validate(pkt)
        assert pkt[3] == 0x09       # command byte
        assert len(pkt) == 9        # 5 + 4

    def test_ip_encoding(self):
        # 172.16.10.2 little-endian: 02 0A 10 AC
        pkt = heartbeat("172.16.10.2")
        assert pkt[4:8] == bytes([0x02, 0x0A, 0x10, 0xAC])

    def test_different_ip(self):
        pkt = heartbeat("172.16.10.100")
        assert pkt[4:8] == bytes([0x64, 0x0A, 0x10, 0xAC])


class TestVideoRequest:
    def test_structure(self):
        pkt = video_request("172.16.10.2")
        assert validate(pkt)
        assert pkt[3] == 0x08

    def test_same_ip_encoding_as_heartbeat(self):
        ip = "172.16.10.50"
        hb = heartbeat(ip)
        vr = video_request(ip)
        assert hb[4:8] == vr[4:8]   # IP bytes identical, only cmd differs
        assert hb[3] != vr[3]


class TestControlPacket:
    def test_neutral_known_bytes(self):
        # protocol.md §4.4: 5A 55 08 02 00 7F 7F 80 80 20 20 08
        assert CONTROL_NEUTRAL == bytes([
            0x5A, 0x55, 0x08, 0x02, 0x00,
            0x7F, 0x7F, 0x80, 0x80, 0x20, 0x20, 0x08
        ])

    def test_neutral_is_valid(self):
        assert validate(CONTROL_NEUTRAL)

    def test_takeoff_flag(self):
        pkt = control(takeoff=True)
        assert pkt[4] & 0x01        # bit 0 set
        assert not (pkt[4] & 0x02)  # land not set
        assert validate(pkt)

    def test_land_flag(self):
        pkt = control(land=True)
        assert pkt[4] & 0x02
        assert validate(pkt)

    def test_rth_flag(self):
        pkt = control(rth=True)
        assert pkt[4] & 0x04
        assert validate(pkt)

    def test_stop_flag(self):
        pkt = control(stop=True)
        assert pkt[4] & 0x80
        assert validate(pkt)

    def test_rotate_encoding(self):
        pkt = control(rotate=64)
        assert pkt[8] == 128        # 64 * 2

        pkt2 = control(rotate=0)
        assert pkt2[8] == 0

        pkt3 = control(rotate=127)
        assert pkt3[8] == 254

    def test_fixed_bytes(self):
        pkt = control()
        assert pkt[5] == 0x7F
        assert pkt[6] == 0x7F
        assert pkt[7] == 0x80
        assert pkt[9]  == 0x20
        assert pkt[10] == 0x20

    def test_length(self):
        assert len(CONTROL_NEUTRAL) == 12