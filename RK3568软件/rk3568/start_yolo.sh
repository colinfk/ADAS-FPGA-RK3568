#!/bin/sh
# Start only after the existing PCIe/HDMI runtime is ready; never unload a busy driver.
set -eu
test "$(id -u)" = 0 || { echo 'Run from sudo -i' >&2; exit 1; }
src=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
case "${ADAS_RUNTIME_MODE:-adas}" in
 adas|passthrough) ;;
 yolo_only)
  test -s "$src/npu_runtime/models/best_rk3568_int8_rkopt.rknn" || { echo 'YOLO_MODEL_MISSING: npu_runtime/models/best_rk3568_int8_rkopt.rknn' >&2; exit 1; }
  test -s "$src/npu_runtime/vendor/librknnrt.so" || { echo 'YOLO_LIBRARY_MISSING: npu_runtime/vendor/librknnrt.so' >&2; exit 1; }
  ;;
 *) echo 'Unknown ADAS_RUNTIME_MODE' >&2; exit 2 ;;
esac
test -e /dev/adas_video0 || { echo 'Prepare the existing PCIe runtime first: sh prepare_after_jtag.sh' >&2; exit 1; }
command -v fuser >/dev/null
if fuser -s /dev/adas_video0; then
    echo 'Stop the current player/capture before starting YOLO; device has an owner' >&2
    exit 1
fi
if ! python3 - "$src" <<'PY'
import json,sys
from pathlib import Path
sys.path.insert(0,sys.argv[1])
import read_vid1_status as vid
bdf=(Path('/sys/class/misc/adas_video0/device').resolve()).name
vid.check_resources(bdf)
status=vid.read_status(bdf)
geometry=vid.read_hdmi_geometry(bdf)
light=vid.read_low_light_status(bdf)
transport=vid.read_feature_transport(bdf)
if not status[1]&1 or not status[1]&(1<<25) or not geometry['mode_valid'] or not light['map_valid'] or not light['output_ready']:
    raise SystemExit('HDMI/DDR/enhancement not ready; run prepare_after_jtag.sh and inspect input')
if transport['version']!=2 or transport['source_kind']!=0:
    raise SystemExit('Expected extended FEAT/FVID/LUXF/RFX1 external HDMI FPGA transport v2')
print('YOLO_PREFLIGHT '+json.dumps(dict(bdf=bdf,enhancement=light,transport=transport)))
PY
then
    echo 'INPUT_PREFLIGHT_FAILED: HDMI/DDR/FPGA status unavailable; check video input and the existing FPGA configuration. No video was displayed.' >&2
    exit 1
fi
hdmi_id=
for status in /sys/class/drm/card*-HDMI-A-*/status; do
    test -f "$status" || continue
    test "$(cat "$status")" = connected || continue
    name=$(basename "$(dirname "$status")")
    card=${name%%-*}
    port=${name#*-}
    index=${card#card}
    state=/sys/kernel/debug/dri/$index/state
    test -r "$state" || continue
    hdmi_id=$(sed -n "s/^[[:space:]]*connector\[\([0-9][0-9]*\)\]: $port[[:space:]]*$/\1/p" "$state" | head -n 1)
    test -z "$hdmi_id" || break
done
test -n "$hdmi_id" || { echo 'Connected HDMI connector ID not found; inspect DRM debugfs or use direct --connector-id' >&2; exit 1; }
cd "$src/npu_runtime"
arch=$(uname -m)
test "$arch" = aarch64 || { echo 'This launcher requires RK3568 64-bit Linux (aarch64)' >&2; exit 1; }
# An explicit experimental executable is isolated from all default launch paths.
if [ -n "${ADAS_RUNTIME_EXECUTABLE:-}" ]; then
    test -x "$ADAS_RUNTIME_EXECUTABLE" || { echo "EXPERIMENT_EXECUTABLE_MISSING: $ADAS_RUNTIME_EXECUTABLE" >&2; exit 1; }
    exec "$ADAS_RUNTIME_EXECUTABLE" --preprocess rga --connector-id "$hdmi_id" "$@"
fi
# Prefer native build when gcc is present. The bundled ARM executable is also available.
if command -v cc >/dev/null && command -v make >/dev/null; then
    # A copied workspace may contain the WSL/x86 test executable.
    if readelf -h ./play_adas_yolo 2>/dev/null | grep -q AArch64; then
        make play_adas_yolo
    else
        make -B play_adas_yolo
    fi
    executable=./play_adas_yolo
else
    executable=./bin/play_adas_yolo_aarch64
fi
exec "$executable" --preprocess rga --connector-id "$hdmi_id" "$@"
