#!/bin/sh
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
desktop=${1:-/home/linaro/Desktop}
test "$(id -u)" = 0 || { echo 'Run as root'; exit 1; }
test -d "$desktop" || { echo "Desktop missing: $desktop"; exit 1; }
for mode in passthrough yolo_only; do
 case "$mode" in passthrough) name='ADAS 视频直通'; icon=video-display ;; yolo_only) name='ADAS YOLO 检测'; icon=camera-video ;; esac
 entry="$desktop/ADAS_${mode}.desktop"
 cat >"$entry" <<EOF
[Desktop Entry]
Version=1.0
Type=Application
Name=$name
Comment=启动 $mode，关闭终端前保留错误日志
Exec=sudo -n sh "$root/launcher/scripts/shortcut_desktop.sh" $mode
Icon=$icon
Terminal=true
StartupNotify=false
Categories=AudioVideo;
EOF
 chmod 755 "$entry"
 chown --reference="$desktop" "$entry"
done
