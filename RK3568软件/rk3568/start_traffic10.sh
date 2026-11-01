#!/bin/sh
set -eu
src=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
model="$src/npu_runtime/models/traffic10_rk3568_int8_rkopt.rknn"
player="$src/npu_runtime/bin/play_traffic10_aarch64"
test -s "$model" || { echo "MODEL10_MISSING: $model" >&2; exit 1; }
test -x "$player" || { echo "MODEL10_PLAYER_MISSING: $player" >&2; exit 1; }
export ADAS_HUD_MODE=intersection ADAS_RUNTIME_EXECUTABLE="$player"
exec sh "$src/start_fpga_key_demo.sh" --model "$model" --model10 "$@"
