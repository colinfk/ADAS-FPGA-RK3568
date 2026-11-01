#!/bin/sh
set -eu
export ADAS_HUD_MODE=intersection
# Keep replay and the explicitly selected six-class fallback on the original path.
if [ "${ADAS_INPUT:-device}" != device ] || [ "${ADAS_INTERSECTION_LEGACY:-0}" = 1 ]; then
    exec sh "$(dirname -- "$0")/_start_runtime.sh" "$@"
fi
src=${ADAS_RK_SCRIPTS:-}
if [ -z "$src" ] && [ -n "${ADAS_RUNTIME_DIR:-}" ]; then src=$(dirname -- "$ADAS_RUNTIME_DIR"); fi
: "${src:?Set ADAS_RK_SCRIPTS to the deployed rk3568 directory}"
test -f "$src/start_traffic10_final.sh" || { echo "MODEL10_LAUNCHER_MISSING: $src/start_traffic10_final.sh" >&2; exit 2; }
set -f
exec sh "$src/start_traffic10_final.sh" --preprocess cpu --lane-mode adaptive --signal-zoom --zoom-every 8 ${ADAS_EXTRA_ARGS:-} "$@"
