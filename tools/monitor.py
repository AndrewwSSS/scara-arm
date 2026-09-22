#!/usr/bin/env python3
"""Serial monitor that does the line editing on this laptop.

`pio device monitor` hands every keystroke straight to the board, so the board
has to echo and handle backspace itself. This does the opposite: the line is
composed here with full readline editing - you see what you type, backspace
erases, arrow keys move, up/down recall history - and nothing is sent until
you press Enter. The sketch stays silent about input and only prints replies.

    python3 tools/monitor.py [port] [baud]

Port defaults to the first /dev/cu.usbserial* (or /dev/ttyUSB* on Linux).
Ctrl+D or Ctrl+C quits. Close it before uploading - the port allows one owner.
"""
import glob
import os
import sys
import threading

import readline  # noqa: F401  - importing it is what enables line editing

BAUD = 115200
REEXEC_FLAG = "MONITOR_REEXEC"


def ensure_pyserial():
    """Re-exec under an interpreter that actually has pyserial.

    The name `serial` on PyPI belongs to an unrelated serialization package, so
    a Python that has that one installed imports fine and then has no
    serial.Serial. PlatformIO ships a venv with real pyserial, so borrow that
    rather than changing the user's environment.
    """
    try:
        import serial

        if hasattr(serial, "Serial"):
            return
    except ImportError:
        pass

    if os.environ.get(REEXEC_FLAG):
        sys.exit(f"pyserial not available under {sys.executable}")

    pio_python = os.path.expanduser("~/.platformio/penv/bin/python")
    if not os.path.exists(pio_python):
        sys.exit(
            "pyserial not found. Either install it (pip install pyserial - note "
            "the PyPI package called `serial` is a different project and will "
            "shadow it) or run this with PlatformIO's interpreter."
        )

    os.environ[REEXEC_FLAG] = "1"
    os.execv(pio_python, [pio_python] + sys.argv)


ensure_pyserial()

import serial  # noqa: E402  - only importable once the check above has passed


def find_port():
    for pattern in ("/dev/cu.usbserial*", "/dev/cu.usbmodem*", "/dev/ttyUSB*", "/dev/ttyACM*"):
        found = sorted(glob.glob(pattern))
        if found:
            return found[0]
    sys.exit("no serial port found - pass one explicitly")


def pump(ser):
    """Print whatever the board says, as it says it."""
    while True:
        try:
            line = ser.readline()
        except (serial.SerialException, OSError):
            return
        if line:
            # \r first so the line starts at column 0 even if a prompt is part-typed
            sys.stdout.write("\r" + line.decode("utf-8", "replace").rstrip("\r\n") + "\n")
            sys.stdout.flush()


def main():
    port = sys.argv[1] if len(sys.argv) > 1 else find_port()
    baud = int(sys.argv[2]) if len(sys.argv) > 2 else BAUD

    # Opening the port toggles DTR, which resets the board - that is why the
    # banner appears a moment after connecting.
    ser = serial.Serial(port, baud, timeout=0.2)
    threading.Thread(target=pump, args=(ser,), daemon=True).start()
    print(f"connected {port} @ {baud} - Ctrl+D to quit")

    while True:
        try:
            line = input()
        except (EOFError, KeyboardInterrupt):
            break
        try:
            ser.write((line + "\n").encode())
        except (serial.SerialException, OSError) as exc:
            print(f"write failed: {exc}")
            break

    ser.close()
    print("\ndisconnected")


if __name__ == "__main__":
    main()
