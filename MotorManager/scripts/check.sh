#!/usr/bin/env bash
# Builds every env in this project's platformio.ini. Run it after a change to
# MotorControl too: the library is linked in through symlink://, so this is
# what catches a library change that breaks this sketch.
#
#   scripts/check.sh            from anywhere inside the project
#   PIO=/path/to/pio scripts/check.sh

set -euo pipefail

cd "$(dirname "$0")/.."

# pio is often not on PATH when installed by the VS Code extension.
PIO="${PIO:-$(command -v pio || echo "$HOME/.platformio/penv/bin/pio")}"

"$PIO" run
