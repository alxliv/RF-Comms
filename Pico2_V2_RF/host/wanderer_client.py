"""Minimal host client for the Pico2_V2_RF base binary frame protocol.

Requires: pip install pyserial

Usage:
    python wanderer_client.py COM5

Commands at the prompt: arm, stop, move L R, getver, getstat, quit
Telemetry, version/status replies, and link-lost notices from the base are
printed as they arrive on a background reader thread. Query answers
(getver/getstat) come back asynchronously on the downlink stream, not as a
synchronous reply to the command. Plain text lines from the base's text
CLI/diagnostics (still active on the same USB CDC link) are printed as-is
for visibility.

Wire format (must match Pico2_V2_RF/src/main.cpp and protocol.h):
    Byte 0        FRAME_SYNC = 0xAA
    Byte 1        Payload length N
    Bytes 2..N+1  Payload (one of the structs below)
    Bytes N+2..N+3 CRC16-CCITT over [length byte + payload], little-endian

CRC16-CCITT: init 0xFFFF, poly 0x1021, MSB-first (same algorithm as
Pico2_V1_RF/src/protocol.c and the base firmware).
"""

import shutil
import struct
import sys
import threading
import time

import serial

FRAME_SYNC = 0xAA
MAX_PAYLOAD_SIZE = 32

CMD_NOP = 0x00
CMD_STOP = 0x10
CMD_ARM = 0x11
CMD_MOVE = 0x12
CMD_GETVER = 0x20
CMD_GETSTAT = 0x21

REPLY_LINK_LOST = 0x00
REPLY_TELEMETRY = 0x01
REPLY_VERSION = 0x02
REPLY_STAT = 0x03

# Telemetry/stat flag bits (rf_protocol::TelemetryFlag).
WAND_MOVING = 1 << 0
WAND_ARMED = 1 << 1

# Vehicle FSM state (rf_protocol::TacticalState) carried in telemetry.
TACTICAL_STATE_NAMES = {
    0: "Safe",
    1: "Active",
    2: "Fallback",
    3: "Fault",
}

COMMAND_HEADER = struct.Struct("<BB")
MOVE_COMMAND = struct.Struct("<BBhh")
TELEMETRY = struct.Struct("<BBBB")
VERSION_REPLY = struct.Struct("<BBB")
STAT_REPLY = struct.Struct("<BBhh")
LINK_LOST_NOTICE = struct.Struct("<B")

COMMAND_NAMES = {
    CMD_NOP: "NOP",
    CMD_STOP: "STOP",
    CMD_ARM: "ARM",
    CMD_MOVE: "MOVE",
    CMD_GETVER: "GETVER",
    CMD_GETSTAT: "GETSTAT",
}


def describe_flags(flags: int) -> str:
    return (f"{'armed' if flags & WAND_ARMED else 'disarmed'},"
            f"{'moving' if flags & WAND_MOVING else 'stopped'}")


def describe_tactical_state(state: int) -> str:
    return TACTICAL_STATE_NAMES.get(state, f"0x{state:02x}")


def crc16_ccitt(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def encode_frame(payload: bytes) -> bytes:
    if len(payload) > MAX_PAYLOAD_SIZE:
        raise ValueError("payload exceeds MAX_PAYLOAD_SIZE")
    body = bytes([len(payload)]) + payload
    crc = crc16_ccitt(body)
    return bytes([FRAME_SYNC]) + body + struct.pack("<H", crc)


def decode_payload(payload: bytes) -> str:
    if not payload:
        return "<empty frame>"

    reply_type = payload[0]
    if reply_type == REPLY_TELEMETRY and len(payload) == TELEMETRY.size:
        _, sequence, tactical_state, flags = TELEMETRY.unpack(payload)
        return (f"TELEMETRY seq={sequence} "
                f"state={describe_tactical_state(tactical_state)} "
                f"[{describe_flags(flags)}]")

    if reply_type == REPLY_VERSION and len(payload) == VERSION_REPLY.size:
        _, major, minor = VERSION_REPLY.unpack(payload)
        return f"VERSION {major}.{minor}"

    if reply_type == REPLY_STAT and len(payload) == STAT_REPLY.size:
        _, flags, target_left, target_right = STAT_REPLY.unpack(payload)
        return (f"STAT [{describe_flags(flags)}] "
                f"target=({target_left},{target_right})mm/s")

    if reply_type == REPLY_LINK_LOST and len(payload) == LINK_LOST_NOTICE.size:
        return "LINK LOST"

    return f"UNKNOWN reply_type=0x{reply_type:02x} payload={payload.hex()}"


class FrameReceiver:
    """Mirrors the base's poll_base_usb() byte state machine: separates
    binary frames (starting with FRAME_SYNC) from the base's plain text
    diagnostic lines on the same byte stream."""

    TEXT, LENGTH, PAYLOAD, CRC = range(4)

    def __init__(self):
        self.state = self.TEXT
        self.text_line = bytearray()
        self.frame_length = 0
        self.frame_payload = bytearray()
        self.frame_crc = bytearray()

    def feed(self, byte: int):
        """Returns ("text", line) for a completed plain-text line, ("frame",
        payload) for a CRC-valid binary frame, or None otherwise."""
        if self.state == self.TEXT:
            if byte == FRAME_SYNC:
                self.state = self.LENGTH
            elif byte in (0x0D, 0x0A):
                if self.text_line:
                    line = self.text_line.decode(errors="replace")
                    self.text_line.clear()
                    return ("text", line)
            else:
                self.text_line.append(byte)
            return None

        if self.state == self.LENGTH:
            if byte > MAX_PAYLOAD_SIZE:
                self.state = self.TEXT
                return None
            self.frame_length = byte
            self.frame_payload.clear()
            self.state = self.CRC if byte == 0 else self.PAYLOAD
            return None

        if self.state == self.PAYLOAD:
            self.frame_payload.append(byte)
            if len(self.frame_payload) == self.frame_length:
                self.frame_crc.clear()
                self.state = self.CRC
            return None

        # self.state == self.CRC
        self.frame_crc.append(byte)
        if len(self.frame_crc) < 2:
            return None

        self.state = self.TEXT
        expected = self.frame_crc[0] | (self.frame_crc[1] << 8)
        body = bytes([self.frame_length]) + bytes(self.frame_payload)
        if expected == crc16_ccitt(body):
            return ("frame", bytes(self.frame_payload))
        return None  # CRC mismatch: drop the frame, resync on next FRAME_SYNC


# Telemetry arrives ~100x/sec, far faster than a human can read scrolling
# lines, so it's redrawn in place on one line instead of being printed.
STATUS_REDRAW_INTERVAL = 0.1  # seconds; caps the redraw rate to ~10 Hz


class StatusLine:
    """Renders fast-changing telemetry on a single, in-place terminal line
    while other output (text diagnostics, replies, link-lost) still scrolls
    normally above it."""

    def __init__(self):
        self._lock = threading.Lock()
        self._text = ""
        self._last_draw = 0.0

    @staticmethod
    def _fit_to_width(text: str) -> str:
        # Truncate to the terminal width so the line can never wrap; a
        # wrapped status line breaks the "\r"/clear-line redraw below,
        # since that only ever addresses the cursor's current row.
        width = shutil.get_terminal_size(fallback=(80, 24)).columns
        return text[: width - 1] if width > 1 else text

    def update(self, text: str):
        now = time.monotonic()
        with self._lock:
            if now - self._last_draw < STATUS_REDRAW_INTERVAL:
                return
            self._draw(text)

    def _draw(self, text: str):
        text = self._fit_to_width(text)
        sys.stdout.write("\r\x1b[2K" + text)
        sys.stdout.flush()
        self._text = text
        self._last_draw = time.monotonic()

    def println(self, text: str):
        with self._lock:
            sys.stdout.write("\r\x1b[2K")
            print(text)
            if self._text:
                sys.stdout.write(self._text)
                sys.stdout.flush()


def reader_thread(port: serial.Serial, receiver: FrameReceiver, status: StatusLine):
    while True:
        data = port.read(1)
        if not data:
            continue
        result = receiver.feed(data[0])
        if result is None:
            continue
        _, value = result
        if isinstance(value, str):
            status.println(f"[base] {value}")
            continue
        if value and value[0] == REPLY_TELEMETRY:
            status.update(decode_payload(value))
        else:
            status.println(decode_payload(value))


def main():
    if len(sys.argv) != 2:
        print(f"Usage: python {sys.argv[0]} <serial-port>")
        sys.exit(1)

    port = serial.Serial(sys.argv[1], baudrate=115200, timeout=0.1)
    receiver = FrameReceiver()
    status = StatusLine()
    threading.Thread(target=reader_thread, args=(port, receiver, status), daemon=True).start()

    sequence = 0
    print("Commands: arm, stop, move L R, getver, getstat, quit")
    while True:
        try:
            line = input("> ").strip()
        except EOFError:
            break

        if line == "quit":
            break
        if line == "":
            continue

        sequence = (sequence + 1) & 0xFF

        if line == "arm":
            port.write(encode_frame(COMMAND_HEADER.pack(CMD_ARM, sequence)))
        elif line == "stop":
            port.write(encode_frame(COMMAND_HEADER.pack(CMD_STOP, sequence)))
        elif line == "getver":
            port.write(encode_frame(COMMAND_HEADER.pack(CMD_GETVER, sequence)))
        elif line == "getstat":
            port.write(encode_frame(COMMAND_HEADER.pack(CMD_GETSTAT, sequence)))
        elif line.startswith("move "):
            parts = line.split()
            if len(parts) != 3:
                print("Usage: move L R")
                continue
            try:
                left, right = int(parts[1]), int(parts[2])
            except ValueError:
                print("Usage: move L R; values must be integers")
                continue
            if not (-32768 <= left <= 32767 and -32768 <= right <= 32767):
                print("Usage: move L R; values must fit signed 16-bit")
                continue
            port.write(encode_frame(MOVE_COMMAND.pack(CMD_MOVE, sequence, left, right)))
        else:
            print("Unknown command. Commands: arm, stop, move L R, getver, getstat, quit")

    port.close()


if __name__ == "__main__":
    main()
