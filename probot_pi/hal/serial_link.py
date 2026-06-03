"""UART transport + COBS + CRC16 framing — a byte-for-byte mirror of the ESP
side (components/services/src/protocol.c + task_comm.c/task_telemetry.c).

Frame on the wire (both directions):
    [0x00][ COBS( payload || crc16_le ) ][0x00]

`import serial` is guarded so this module (CRC/COBS/struct) stays importable for
host unit tests where pyserial is not installed.
"""
import struct
import threading

from probot_pi.bsp import params as P

try:
    import serial  # pyserial
except ImportError:  # pragma: no cover - host test without hardware deps
    serial = None


# --------------------------------------------------------------------------- #
# CRC16-CCITT (poly 0x1021, init 0xFFFF, no reflect, no xorout) — protocol.c   #
# --------------------------------------------------------------------------- #
def crc16_ccitt(data):
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if (crc & 0x8000) else (crc << 1) & 0xFFFF
    return crc & 0xFFFF


# --------------------------------------------------------------------------- #
# COBS — standard, eliminates 0x00. Matches cobs_encode/decode in protocol.c.  #
# --------------------------------------------------------------------------- #
def cobs_encode(data):
    out = bytearray([0])          # placeholder for the first code byte
    code_idx = 0
    code = 1
    for b in data:
        if b == 0:
            out[code_idx] = code
            code = 1
            code_idx = len(out)
            out.append(0)
        else:
            out.append(b)
            code += 1
            if code == 0xFF:      # max run length -> close the block
                out[code_idx] = code
                code = 1
                code_idx = len(out)
                out.append(0)
    out[code_idx] = code
    return bytes(out)


def cobs_decode(data):
    out = bytearray()
    i, n = 0, len(data)
    while i < n:
        code = data[i]
        i += 1
        if code == 0:
            return b""            # invalid: 0x00 inside a COBS block
        for _ in range(1, code):
            if i >= n:
                break
            out.append(data[i])
            i += 1
        if code != 0xFF and i < n:
            out.append(0)
    return bytes(out)


# --------------------------------------------------------------------------- #
# Packet (un)packing                                                          #
# --------------------------------------------------------------------------- #
CMD_SIZE   = struct.calcsize(P.CMD_FMT)     # 11
TELEM_SIZE = struct.calcsize(P.TELEM_FMT)   # 32


def pack_cmd(omega_ref_l, omega_ref_r, mode, seq):
    """Build a full on-the-wire command frame (delimiters included)."""
    payload = struct.pack(P.CMD_FMT, float(omega_ref_l), float(omega_ref_r),
                          int(mode) & 0xFF, int(seq) & 0xFFFF)
    body = payload + struct.pack("<H", crc16_ccitt(payload))
    return bytes([P.PROTO_DELIM]) + cobs_encode(body) + bytes([P.PROTO_DELIM])


def parse_telem(body):
    """COBS-decoded body (payload||crc16_le) -> telem dict, or None if invalid."""
    if len(body) != TELEM_SIZE + 2:
        return None
    payload, crc_rx = body[:TELEM_SIZE], body[TELEM_SIZE:]
    if crc16_ccitt(payload) != struct.unpack("<H", crc_rx)[0]:
        return None
    wl, wr, yaw, yr, pl, pr, vbat, faults, seq = struct.unpack(P.TELEM_FMT, payload)
    return {
        "omega_meas_l": wl, "omega_meas_r": wr,
        "yaw": yaw, "yaw_rate": yr,         # radians, rad/s (see robot_state.h)
        "pwm_l": pl, "pwm_r": pr, "vbat": vbat,
        "fault_flags": faults, "seq": seq,
    }


class FrameReader:
    """Accumulate a byte stream and split it into frames on PROTO_DELIM."""

    def __init__(self, max_frame=128):
        self._acc = bytearray()
        self._max = max_frame

    def feed(self, chunk):
        frames = []
        for b in chunk:
            if b == P.PROTO_DELIM:
                if self._acc:
                    frames.append(bytes(self._acc))
                    self._acc.clear()
            elif len(self._acc) < self._max:
                self._acc.append(b)
            else:
                self._acc.clear()        # overflow -> drop, resync at next delim
        return frames


# --------------------------------------------------------------------------- #
# Live link                                                                   #
# --------------------------------------------------------------------------- #
class SerialLink:
    """Full-duplex link to the ESP. A background thread reads + decodes frames
    and hands each telemetry dict to `on_telem`; `send_cmd` writes commands."""

    def __init__(self, port=P.PI_UART_PORT, baud=P.PI_UART_BAUD, on_telem=None):
        if serial is None:
            raise RuntimeError("pyserial not installed (pip install pyserial)")
        self._ser = serial.Serial(port, baud, timeout=0.02)
        self._reader = FrameReader()
        self._on_telem = on_telem
        self._thread = None
        self._running = False
        self.stats = {"rx_frames": 0, "rx_bad": 0, "tx_frames": 0}

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._rx_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        if self._thread:
            self._thread.join(timeout=0.5)
        try:
            self._ser.close()
        except Exception:
            pass

    def send_cmd(self, omega_ref_l, omega_ref_r, mode, seq):
        self._ser.write(pack_cmd(omega_ref_l, omega_ref_r, mode, seq))
        self.stats["tx_frames"] += 1

    def _rx_loop(self):
        while self._running:
            data = self._ser.read(256)
            if not data:
                continue
            for body in self._reader.feed(data):
                telem = parse_telem(cobs_decode(body))
                if telem is None:
                    self.stats["rx_bad"] += 1
                else:
                    self.stats["rx_frames"] += 1
                    if self._on_telem:
                        self._on_telem(telem)