#!/bin/sh
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
old=$(cat "$root/deployment_state/previous_driver_release.txt")
case "$old" in /opt/adas-pcie/releases/*) ;; *) echo 'Invalid saved release'; exit 1;; esac
test -f "$old/adas_video_pcie.ko"
if [ -e /dev/adas_video0 ] && fuser -s /dev/adas_video0; then echo 'Stop current capture first'; exit 1; fi
if [ -d /sys/module/adas_video_pcie ]; then rmmod adas_video_pcie; fi
insmod "$old/adas_video_pcie.ko" cached_frames=1
ln -s "$old" /opt/adas-pcie/current.rollback.$$
mv -Tf /opt/adas-pcie/current.rollback.$$ /opt/adas-pcie/current
echo 'Prior driver restored; its software version should be used with it'
