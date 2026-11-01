#!/bin/sh
# Physical requests only; keyboard lamp/speed overrides remain opt-in elsewhere.
set -eu
src=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
exec sh "$src/start_roi4.sh" --fpga-keys "$@"
