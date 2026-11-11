#!/usr/bin/env bash
# Runs the host tests and builds the sketch for both boards. With a
# platformio.local.ini, it builds against the local library checkouts, so run
# it after a library change too.
#
#   scripts/check.sh            from anywhere inside the project
#   PIO=/path/to/pio scripts/check.sh

set -euo pipefail

cd "$(dirname "$0")/.."

# pio is often not on PATH when installed by the VS Code extension.
PIO="${PIO:-$(command -v pio || echo "$HOME/.platformio/penv/bin/pio")}"

"$PIO" test -e native
"$PIO" run -e uno
"$PIO" run -e esp32
