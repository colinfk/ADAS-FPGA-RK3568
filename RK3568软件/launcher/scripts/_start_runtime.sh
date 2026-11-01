#!/bin/sh
set -eu
# Reuse the tested physical HDMI/BAR/key guards; do not replace board plumbing.
if [ "${ADAS_INPUT:-device}" = device ]; then
    src=${ADAS_RK_SCRIPTS:-}
    if [ -z "$src" ] && [ -n "${ADAS_RUNTIME_DIR:-}" ]; then src=$(dirname -- "$ADAS_RUNTIME_DIR"); fi
    : "${src:?Set ADAS_RK_SCRIPTS to the deployed rk3568 directory}"
    test -f "$src/start_fpga_key_demo.sh" || { echo 'Missing guarded FPGA launcher' >&2; exit 2; }
    if [ "${ADAS_HUD_MODE:-}" = surround ]; then set -- --quad-composite --quad-layout rear-quarter "$@"; fi
    set -f
    exec sh "$src/start_fpga_key_demo.sh" --preprocess cpu --lane-mode adaptive ${ADAS_EXTRA_ARGS:-} "$@"
fi
: "${ADAS_RUNTIME_DIR:?Set ADAS_RUNTIME_DIR for a binary RGB565 replay}"
case "$ADAS_INPUT" in
    replay:*) set -- --replay "${ADAS_INPUT#replay:}" "$@" ;;
    *) echo 'ADAS_INPUT must be device or replay:/absolute/RGB565_META_FILE' >&2; exit 2 ;;
esac
cd -- "$ADAS_RUNTIME_DIR"
if [ "${ADAS_HUD_MODE:-}" = surround ]; then set -- --quad-composite --quad-layout rear-quarter "$@"; fi
set -f
exec ./play_adas_yolo --preprocess cpu --lane-mode adaptive ${ADAS_EXTRA_ARGS:-} "$@"
