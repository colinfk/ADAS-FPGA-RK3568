#!/bin/sh
# Start the trained lane v2 fusion after the existing V10 PCIe runtime is ready.
set -eu
for arg in "$@"; do
    if [ "$arg" = --interactive ]; then
        echo 'WARNING: COMPETITION DEMO FORBIDS --interactive: Q/E/R/W can fake FORCE/red/motion.' >&2
        if [ "${ADAS_ALLOW_INTERACTIVE:-0}" != 1 ]; then
            echo 'REJECTED: lab use requires explicit ADAS_ALLOW_INTERACTIVE=1.' >&2
            exit 2
        fi
        echo 'LAB ONLY: interactive overrides explicitly enabled; do not use in a judged run.' >&2
    fi
done
src=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
test "$(uname -m)" = aarch64 || { echo 'Requires RK3568 aarch64 Linux' >&2; exit 1; }
test -e /sys/class/misc/adas_video0/device || { echo 'Run the existing PCIe preparation first' >&2; exit 1; }
bdf=$(basename "$(readlink -f /sys/class/misc/adas_video0/device)")
bar=/sys/bus/pci/devices/$bdf/resource0
python3 - "$src" "$bar" <<'PY'
import mmap,struct,sys
from pathlib import Path
sys.path.insert(0,sys.argv[1])
from lane_bar_read_v2 import read_snapshot
from read_vid1_status import check_resources
check_resources(Path(sys.argv[2]).parent.name)
with open(sys.argv[2],'rb',buffering=0) as f:
    with mmap.mmap(f.fileno(),65536,flags=mmap.MAP_SHARED,prot=mmap.PROT_READ) as bar:
        s=read_snapshot(lambda offset:struct.unpack_from('<I',bar,offset)[0])
if s['model_version']!=2 or s['roi_config_version'] not in (3,4):
    raise SystemExit('Expected trained model v2 / ROI v3 or v4 at BAR FEC0')
print('TINY_CNN_PREFLIGHT',s)
PY
# The existing launcher checks exclusive device ownership and discovers HDMI ID.
exec sh "$src/start_yolo.sh" --preprocess cpu --cnn-bar "$bar" --lane-mode adaptive "$@"
