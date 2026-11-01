#!/bin/sh
# Prepare PCIe and bind the matching driver. Does not start video playback.
set -eu
cd /opt/adas-pcie/current
printf 'ADAS_BOOT_ID=%s KERNEL=%s\n' "$(cat /proc/sys/kernel/random/boot_id)" "$(uname -r)"
state=$(python3 ./prepare_adas_pcie.py --repair --wait-seconds 20)
printf '%s\n' "$state"
bdf=$(printf '%s\n' "$state" | python3 -c 'import json,sys; print(json.load(sys.stdin)["bdf"])')
if [ ! -d /sys/module/adas_video_pcie ]; then
    insmod ./adas_video_pcie.ko cached_frames=1
elif [ "$(cat /sys/module/adas_video_pcie/version)" != "5.2-marker-ack" ]; then
    echo 'A different ADAS driver is loaded; stop playback and unload it first' >&2
    exit 1
fi
if [ ! -L "/sys/bus/pci/devices/$bdf/driver" ]; then
    printf '%s' "$bdf" > /sys/bus/pci/drivers/adas_video/bind
fi
test -c /dev/adas_video0
python3 ./read_vid1_status.py "$bdf" --check-bars --dma --low-light --require-low-light-version 4
