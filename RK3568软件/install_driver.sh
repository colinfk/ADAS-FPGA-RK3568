#!/bin/sh
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
test "$(id -u)" = 0 || { echo 'Use sudo -i'; exit 1; }
test "$(uname -m)" = aarch64
if [ "$(cat /sys/module/adas_video_pcie/version 2>/dev/null || true)" = 5.2-marker-ack ]; then
 echo '5.2-marker-ack already loaded; keeping current driver'; exit 0
fi
if [ -e /dev/adas_video0 ] && fuser -s /dev/adas_video0; then
 echo 'Return to launcher home and stop current capture before installing'; exit 1
fi
kdir=${KDIR:-/lib/modules/$(uname -r)/build}
if [ ! -d "$kdir" ] && [ -d /usr/src/linux-headers-6.1-rockchip ]; then kdir=/usr/src/linux-headers-6.1-rockchip; fi
test -d "$kdir" || { echo 'Matching running-kernel headers required; set KDIR'; exit 1; }
test "$(make -s -C "$kdir" kernelrelease)" = "$(uname -r)" || { echo 'Kernel/header version mismatch'; exit 1; }
make -C "$root/driver" KDIR="$kdir" all
release=/opt/adas-pcie/releases/marker-ack-20261021-$(sha256sum "$root/driver/adas_video_pcie.c" | cut -c1-12)
mkdir -p "$release" "$root/deployment_state"
for f in adas_video_pcie.c adas_video_pcie.ko Makefile prepare_adas_pcie.py read_vid1_status.py start_adas_pcie.sh adas-pcie-prepare.service; do
 cp "$root/driver/$f" "$release/$f"
done
old=$(readlink -f /opt/adas-pcie/current 2>/dev/null || true)
printf '%s\n' "$old" > "$root/deployment_state/previous_driver_release.txt"
python3 "$release/prepare_adas_pcie.py" --repair --wait-seconds 20
if [ -d /sys/module/adas_video_pcie ]; then rmmod adas_video_pcie; fi
if ! insmod "$release/adas_video_pcie.ko" cached_frames=1; then
 if [ -n "$old" ] && [ -f "$old/adas_video_pcie.ko" ]; then insmod "$old/adas_video_pcie.ko" cached_frames=1; fi
 echo 'New driver failed; prior module reload attempted'; exit 1
fi
ln -s "$release" /opt/adas-pcie/current.new.$$
mv -Tf /opt/adas-pcie/current.new.$$ /opt/adas-pcie/current
sh "$release/start_adas_pcie.sh"
echo 'Driver installed; old release retained for rollback'
