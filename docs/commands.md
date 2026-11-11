# Console commands

The board reads commands over USB serial at 115200 baud, one per line. Use
[`tools/monitor.py`](../tools/monitor.py), the
[browser console](console.md), or any serial monitor. `?` on the board prints
this list.

Arguments are checked before a command runs: a missing number or a word where a
number belongs prints a usage line instead.

## Selecting axes

Most commands act on the selected axis.

| Command | Does |
| --- | --- |
| `x` `y` `z` `a` | select that axis |
| `*` | select every axis |

## Moving

| Command | Does |
| --- | --- |
| `f [n]` | jog n steps forward (DIR high); 100 by default |
| `b [n]` | jog n steps backward (DIR low); 100 by default |
| `g <n>` | go to absolute step n (may be negative) |
| `h` | halt: cancel the move and hold position |
| `o` | make the current position step 0 |
| `home` | home against the endstop |
| `grip [n]` | turn the gripper to n degrees, 0–180; bare `grip` shows where it is |

- `home` blocks: the console does not answer until every selected axis is done.
  Axes without a switch on their homing side are skipped, so `*` then `home`
  homes everything that can be homed.
- `grip` ignores the axis selection, because the gripper is not an axis.

## Tuning

| Command | Does |
| --- | --- |
| `v [n]` | step interval in µs, lower is faster; bare `v` restores the default |
| `r [n]` | ramp over n steps; `r 0` turns it off, bare `r` toggles it |
| `p [n]` | release the coils after n ms with the whole fleet idle; `p 0` never, bare `p` toggles |
| `lim <min> <max>` | soft limits in steps |
| `lim off` | remove the soft limits |
| `autohome [0\|1]` | home every axis with a switch at start-up |

- `v` takes effect on the next pulse, even mid-move. There is no ramp between
  the two speeds, so a big jump under load can make the motor lose steps.
- `r`: the ramp length is shared by every axis on the ramp, so `r <n>` changes
  it for all of them. The selection decides which axes use the ramp at all.
- `p` is fleet-wide and ignores the selection, because the CNC Shield has one
  EN line for all sockets. A bare `p` toggles, which is a quick way to feel the
  difference on the shaft. Turning it off energises the drivers again at once.
- `lim` takes effect only once the position is known, after `home` or `o`.
  Before that, a limit would be measured from an unknown point.
- `autohome` without an argument only shows the setting, so a stray command
  cannot make the arm start moving by itself at power-up. It is off by default.

## Settings

| Command | Does |
| --- | --- |
| `save` | keep the current settings (`v`, `r`, `p`, `lim`, `autohome`) across power cycles |
| `load` | go back to the saved settings |
| `defaults` | go back to the built-in settings; what is saved is not touched, so `save` afterwards to keep them |

`s` shows whether there are unsaved changes. How settings are stored:
[settings.md](settings.md).

## Diagnostics

| Command | Does |
| --- | --- |
| `s` | status of every axis |
| `d` | toggle debug output for the selected axes |
| `l` | read each endstop pin now, and how the wiring mode interprets it; moves nothing |
| `mem` | free RAM |
| `reset` | restart the board |
| `?`, `help` | the command list |

- `mem` prints two numbers. On the Uno, **free now** is the gap between the
  heap and the stack at this moment, and **never used** is how much of that gap
  nothing has touched since start-up: the margin at the worst moment so far.
  Use the console for a while (`home`, `s`, `l`, …) before reading it. On the
  ESP32 they are the free heap now and its lowest point.
- `reset` halts every axis first, so nothing is mid-pulse when the pins are
  released. The position is lost: home or set the origin again afterwards.
