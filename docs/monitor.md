# Serial monitor

[`tools/monitor.py`](../tools/monitor.py) is a terminal serial monitor for the
[console commands](commands.md).

```sh
python3 tools/monitor.py [port] [baud]
```

- **Port:** defaults to the first `/dev/cu.usbserial*` on macOS,
  `/dev/ttyUSB*` on Linux, or the first USB COM port on Windows.
- **Baud:** 115200 by default.
- **Quit:** Ctrl+D or Ctrl+C.

## Why not `pio device monitor`

`pio device monitor` sends every keystroke to the board as it is typed, so the
board would have to echo characters and handle backspace itself. This monitor
edits the line on the computer instead: you see what you type, backspace and the
arrow keys work, up/down recall history, and nothing is sent until Enter. The
sketch only prints replies.

## Notes

- Opening the port resets the board (DTR toggles), so the start-up banner
  appears a moment after connecting.
- A serial port has one owner at a time: close the monitor before uploading
  firmware.
- It needs [pyserial](https://pypi.org/project/pyserial/). If the current Python
  lacks it, the script restarts itself under PlatformIO's Python, which ships
  with pyserial.
