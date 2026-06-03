"""Host self-test for the wire protocol + kinematics (no skfuzzy/pyserial/numpy
needed). Run from the project root:

    python tests/test_protocol.py

Verifies the Pi framing matches the ESP byte-for-byte using published CRC/COBS
check vectors, plus full cmd/telem round-trips and a kinematics identity.
"""
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from probot_pi.bsp import params as P
from probot_pi.hal import serial_link as sl
from probot_pi.services import kinematics as kin


def check(name, cond):
    print(f"  {'ok  ' if cond else 'FAIL'} {name}")
    if not cond:
        raise AssertionError(name)


def test_crc_vector():
    # CRC-16/CCITT-FALSE published check value for "123456789" is 0x29B1.
    check("crc16(\"123456789\") == 0x29B1", sl.crc16_ccitt(b"123456789") == 0x29B1)


def test_cobs_vector():
    # Canonical COBS example: 11 22 00 33 -> 03 11 22 02 33.
    enc = sl.cobs_encode(bytes([0x11, 0x22, 0x00, 0x33]))
    check("cobs_encode canonical vector", enc == bytes([0x03, 0x11, 0x22, 0x02, 0x33]))
    check("cobs single zero -> 01 01", sl.cobs_encode(b"\x00") == b"\x01\x01")
    # round-trip over payloads that contain zeros (worst case for framing)
    for raw in (b"", b"\x00" * 8, bytes(range(40)), b"\xff" * 300):
        check(f"cobs round-trip len={len(raw)}", sl.cobs_decode(sl.cobs_encode(raw)) == raw)


def test_sizes():
    check("CMD_SIZE == 11", sl.CMD_SIZE == 11)
    check("TELEM_SIZE == 32", sl.TELEM_SIZE == 32)


def test_cmd_roundtrip():
    frame = sl.pack_cmd(1.25, -2.5, P.MODE_RUN, 0x1234)
    check("cmd frame delimited by 0x00", frame[0] == 0x00 and frame[-1] == 0x00)
    check("no 0x00 inside cmd body", 0x00 not in frame[1:-1])
    # decode the way the ESP task_comm does
    body = sl.cobs_decode(frame[1:-1])
    check("decoded cmd len == 13", len(body) == sl.CMD_SIZE + 2)
    payload, crc_rx = body[:sl.CMD_SIZE], struct.unpack("<H", body[sl.CMD_SIZE:])[0]
    check("cmd CRC verifies", sl.crc16_ccitt(payload) == crc_rx)
    wl, wr, mode, seq = struct.unpack(P.CMD_FMT, payload)
    check("cmd fields survive", abs(wl - 1.25) < 1e-6 and abs(wr + 2.5) < 1e-6
          and mode == P.MODE_RUN and seq == 0x1234)


def test_telem_roundtrip():
    # Build a telem frame exactly the way task_telemetry.c does, then parse it.
    payload = struct.pack(P.TELEM_FMT, 3.0, 3.1, 0.5, -0.2, 0.4, 0.45, 11.1,
                          P.FAULT_NONE, 0x00AA)
    framed = bytes([P.PROTO_DELIM]) + sl.cobs_encode(payload + struct.pack(
        "<H", sl.crc16_ccitt(payload))) + bytes([P.PROTO_DELIM])
    reader = sl.FrameReader()
    bodies = reader.feed(framed)
    check("framer extracts exactly one frame", len(bodies) == 1)
    t = sl.parse_telem(sl.cobs_decode(bodies[0]))
    check("telem parses", t is not None)
    check("telem fields survive", abs(t["omega_meas_l"] - 3.0) < 1e-6
          and t["seq"] == 0x00AA and t["fault_flags"] == 0)
    # a corrupted byte must be rejected by CRC
    bad = bytearray(framed)
    bad[3] ^= 0xFF
    rb = sl.FrameReader().feed(bytes(bad))
    check("corrupted telem rejected", sl.parse_telem(sl.cobs_decode(rb[0])) is None)


def test_framer_split():
    # two frames back-to-back plus leading/trailing noise delimiters
    f1 = sl.pack_cmd(1.0, 1.0, P.MODE_RUN, 1)
    f2 = sl.pack_cmd(2.0, 2.0, P.MODE_RUN, 2)
    stream = bytes([0x00]) + f1 + f2 + bytes([0x00])
    bodies = sl.FrameReader().feed(stream)
    check("framer finds both frames", len(bodies) == 2)


def test_kinematics_identity():
    for v, w in ((0.3, 0.0), (0.0, 1.5), (0.25, -0.8)):
        wl, wr = kin.inverse(v, w)
        v2, w2 = kin.forward(wl, wr)
        check(f"inverse->forward identity (v={v}, w={w})",
              abs(v2 - v) < 1e-9 and abs(w2 - w) < 1e-9)
    # pure spin: wheels equal-and-opposite, wheel_yaw_rate matches forward()
    wl, wr = kin.inverse(0.0, 1.0)
    check("spin -> wl == -wr", abs(wl + wr) < 1e-9)
    check("wheel_yaw_rate == forward yaw", abs(kin.wheel_yaw_rate(wl, wr) - 1.0) < 1e-9)


def main():
    print("probot_pi protocol/kinematics self-test")
    for fn in (test_crc_vector, test_cobs_vector, test_sizes, test_cmd_roundtrip,
               test_telem_roundtrip, test_framer_split, test_kinematics_identity):
        print(f"[{fn.__name__}]")
        fn()
    print("\nALL PASS")


if __name__ == "__main__":
    main()
