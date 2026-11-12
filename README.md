# SCARA Arm

[![check](https://github.com/AndrewwSSS/scara-arm/actions/workflows/check.yml/badge.svg)](https://github.com/AndrewwSSS/scara-arm/actions/workflows/check.yml)
[![License: MIT](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
![Platforms](https://img.shields.io/badge/platforms-Arduino%20Uno%20%7C%20ESP32-informational)

Firmware and tools for a 3D-printed, 4-axis SCARA robot arm with a servo
gripper. Four stepper motors are driven from a CNC Shield on an Arduino Uno or
an ESP32, and the arm is controlled from a serial console in the terminal or in
the browser.

<!-- Photo or GIF goes here: ![The arm](docs/media/arm.jpg) -->

## Features

- **Two boards, one sketch.** The same firmware builds for the Arduino Uno and the
  ESP32; each board's pin map is one block in [`src/Board.h`](src/Board.h).
- **Serial console** with over 20 commands: jog, go to a step, home, soft limits,
  speed, ramps, gripper, diagnostics. Arguments are checked before a command runs.
- **Browser control panel** over Web Serial: configurable command buttons,
  automatic reconnect, no install.
- **Homing** against a limit switch, and **soft limits** once the position is known.
- **Smooth motion:** acceleration ramps, and coils released when the arm is idle.
- **Settings survive power cycles,** stored in EEPROM with a version and a CRC, so
  a damaged or outdated save falls back to safe defaults.
- **Tested and CI-built:** host-side unit tests, plus Uno and ESP32 builds on
  every push.

## Hardware

| Part | Used |
| --- | --- |
| Controller | Arduino Uno, or an ESP32 DevKit through a 3.3 V ↔ 5 V level shifter |
| Driver board | CNC Shield v3 with A4988 or DRV8825 drivers, 1/16 microstepping |
| Motors | 4 × 1.8° stepper motors (sockets X, Y, Z, A) |
| Sensors | 1 normally-closed limit switch (X) |
| Gripper | hobby servo with its own 5–6 V supply |
| Frame | 3D-printed parts, steel guide rods and a lead screw |

Pin maps and wiring rules: [docs/wiring.md](docs/wiring.md).

## Getting started

Requires [PlatformIO](https://platformio.org/). The libraries are fetched
automatically.

```sh
git clone https://github.com/AndrewwSSS/scara-arm.git
cd scara-arm
pio run -e uno -t upload        # or: -e esp32
python3 tools/monitor.py        # serial console
```

Then, in the console:

```
?            list the commands
x            select axis X
home         home it against its limit switch
f 3200       jog one motor revolution forward
g 0          go back to step 0
```

Or open [`tools/console.html`](tools/console.html) in Chrome or Edge for the
button panel.

## How it is built

The firmware is a thin layer over two libraries written for this arm:

- [MotorControl](https://github.com/AndrewwSSS/MotorControl): non-blocking
  stepper and servo fleets, endstops, homing, ramps and idle power release.
- [CommandDispatcher](https://github.com/AndrewwSSS/CommandDispatcher): parses
  console lines into commands with checked arguments.

| Path | Contents |
| --- | --- |
| [`src/main.cpp`](src/main.cpp) | the sketch: axes, settings and console commands |
| [`src/Board.h`](src/Board.h) | pin map, driver and step engine per board |
| [`src/SettingsStore.h`](src/SettingsStore.h) | EEPROM storage with version and CRC checks |
| [`tools/`](tools) | browser console, terminal monitor, default button set |
| [`test/`](test) | host-side unit tests against a fake Arduino core |

To work on the libraries alongside the sketch, copy
`platformio.local.ini.example` to `platformio.local.ini`: builds then use the
library folders next to this repo instead of the releases.

## Documentation

| Guide | Covers |
| --- | --- |
| [Console commands](docs/commands.md) | every command and how it behaves |
| [Browser console](docs/console.md) | connecting, reconnecting, buttons |
| [Serial monitor](docs/monitor.md) | the terminal monitor |
| [Wiring](docs/wiring.md) | pin maps for the Uno and the ESP32, switches, servo |
| [Settings storage](docs/settings.md) | the EEPROM record and its checks |

## Tests

```sh
pio test -e native     # unit tests on the host
scripts/check.sh       # unit tests, plus the Uno and ESP32 builds
```

## License

[MIT](LICENSE)
