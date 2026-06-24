"""Host client for the Pico2_V2_RF base text protocol (see PROTOCOL.md).

Requires: pip install pyserial

Usage:
    python wanderer_client.py COM5

The base speaks a line-based ASCII protocol over USB CDC. Commands typed at the
prompt are sent verbatim as bare verbs; the base replies with sigil-prefixed
lines, one message per line:

    =  result (ack)     >  data reply       #  telemetry
    !  event            *  log

Telemetry (`#`) is fast-changing and redrawn in place on one status line;
everything else scrolls above it. Telemetry is off by default -- type `tlm on`
or `tlm <hz>` to start the `#` stream. The base parses and validates every
command, so this client forwards typed lines without re-implementing the
protocol; only `quit` is handled locally.
"""

import shutil
import sys
import threading
import time

import serial


# Telemetry arrives far faster than a human can read scrolling lines, so it's
# redrawn in place on one line instead of being printed.
STATUS_REDRAW_INTERVAL = 0.1  # seconds; caps the redraw rate to ~10 Hz


class StatusLine:
    """Renders fast-changing telemetry on a single, in-place terminal line
    while other output (results, replies, events, logs) still scrolls
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
            # Erase the in-place status line and scroll the message. The status
            # line is intentionally NOT re-pinned afterwards: a live telemetry
            # stream redraws it within STATUS_REDRAW_INTERVAL, whereas a stale
            # line (e.g. after `tlm off`) must not reappear beneath every reply.
            sys.stdout.write("\r\x1b[2K")
            print(text)
            self._text = ""


def reader_thread(port: serial.Serial, status: StatusLine):
    """Reassembles the base's byte stream into lines and routes each by its
    sigil: telemetry redraws the status line, everything else scrolls."""
    buffer = bytearray()
    while True:
        data = port.read(256)
        if not data:
            continue
        buffer.extend(data)
        while b"\n" in buffer:
            raw, _, rest = buffer.partition(b"\n")
            buffer = bytearray(rest)
            line = raw.decode("ascii", errors="replace").strip()
            if not line:
                continue
            if line[0] == "#":
                status.update(line)
            else:
                # `=` results, `>` replies, `!` events, `*` logs, and anything
                # unrecognized all scroll as-is for a human to read.
                status.println(line)


def main():
    comport = sys.argv[1] if len(sys.argv) >= 2 else "COM4"

    port = serial.Serial(comport, baudrate=115200, timeout=0.1)
    status = StatusLine()
    threading.Thread(target=reader_thread, args=(port, status), daemon=True).start()

    print("Commands: arm | stop | move L R | ver | stat | "
          "tlm on|off|<hz> | rf on|off | setbpa 0-3 | setwpa 0-3 | "
          "ping | help | quit")
    print("Telemetry is off by default; type 'tlm on' to start it.")
    print("RF link stats: 'rf' for one report, 'rf on' to repeat once a second.")
    print("PA: 'setbpa <0-3>' sets base power; 'setwpa <0-3>' sets Wanderer "
          "power (replies '>pa=' with the level applied).")
    while True:
        try:
            line = input("> ").strip()
        except EOFError:
            break

        if line == "":
            continue
        if line.lower() == "quit":
            break

        # The base owns parsing and validation; forward the line as typed.
        port.write((line + "\n").encode("ascii", errors="ignore"))

    port.close()


if __name__ == "__main__":
    main()
