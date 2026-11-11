# Wiring

The pins live in [`src/Board.h`](../src/Board.h), one block per board. The rest
of the sketch is shared, so supporting a new board means adding a block there.

## Arduino Uno with the CNC Shield v3

| Socket | STEP | DIR | Limit header | Notes |
| --- | --- | --- | --- | --- |
| X | D2 | D5 | D9 | hardwired by the shield |
| Y | D3 | D6 | D10 | hardwired by the shield |
| Z | D4 | D7 | D11 | hardwired by the shield |
| A | D12 | D13 | none | |

- Shared enable (EN): D8, active LOW.
- X limit switch: **A4**, not the shield's limit header.
- Gripper servo signal: **D11**, the Z limit header, free because Z has no switch.
- The shield's sockets take A4988 or DRV8825 modules only.

## ESP32-WROOM-32 DevKit

The ESP32 drives the same CNC Shield through a 3.3 V ↔ 5 V level shifter.

| Socket | STEP | DIR | Limit |
| --- | --- | --- | --- |
| X | GPIO25 | GPIO26 | GPIO34 |
| Y | GPIO27 | GPIO33 | |
| Z | GPIO32 | GPIO13 | |
| A | GPIO18 | GPIO19 | |

- Shared enable (EN): GPIO4, active LOW. It needs a pull-up to the drivers'
  logic supply, so the drivers stay off while the ESP32 boots.
- Gripper servo signal: GPIO21.
- GPIO34–39 are input-only and have no internal pull-up, so the X switch needs
  an external 10 kΩ pull-up to 3.3 V.
- The pin map avoids the boot strapping pins (0, 2, 5, 12, 15), the flash pins
  (6–11), the USB serial pins (1, 3) and GPIO14, which pulses during boot.

## Limit switches

Each switch is wired **COM → GND** and **NC → pin**, normally closed. The pin
uses its pull-up, so a broken wire reads as a pressed switch and stops the axis.

## Gripper servo

Only the signal comes from the board. The servo needs its own 5–6 V supply,
with GND shared with the board. Set `GRIPPER_ENABLED` to `false` in `Board.h`
when no servo is connected; its pin is then left untouched.

## Drivers and step engines

Each socket names its driver (`X_DRIVER`, …) and step engine (`X_ENGINE`, …)
in `Board.h`:

- **Driver:** `Driver::A4988` (also DRV8825, 2 µs STEP pulses) or
  `Driver::TB6600` (also DM542, opto-isolated, 5 µs pulses). The DIR setup time
  before a reversal is MotorControl's `MC_DIR_SETUP_US`, 5 µs for every driver:
  enough for the slow ones, and negligible for the rest, because it is only
  paid when the direction changes.
- **Engine:** `Engine::POLLED`, MotorControl's `PolledEngine`, which times the
  pulses from `loop()` on any board. Changing a socket's engine is a one-line
  change.
