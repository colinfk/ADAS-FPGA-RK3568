#!/bin/sh
# Keep startup errors visible in the desktop terminal and in a per-run log.
set -u
root=$(CDPATH= cd -- "$(dirname -- "$0")/../.." && pwd)
mode=${1:-}
case "$mode" in passthrough|yolo_only) ;; *) echo 'Expected passthrough or yolo_only'; exit 2 ;; esac
mkdir -p "$root/launcher/logs"
log="$root/launcher/logs/${mode}_$(date +%Y%m%d_%H%M%S)_$$.log"
echo "模式: $mode   日志: $log"
python3 "$root/launcher/scripts/shortcut_runner.py" "$mode" >"$log" 2>&1
rc=$?
if [ "$rc" -eq 0 ]; then exit 0; fi
cat "$log"
echo "退出码: $rc。按 Enter 关闭；运行时可点击屏幕退出或用 Ctrl+C 停止。"
read -r ignored
exit "$rc"
