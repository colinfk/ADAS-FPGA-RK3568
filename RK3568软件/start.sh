#!/bin/sh
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
test "$(id -u)" = 0 || { echo 'Use sudo -i, then sh start.sh'; exit 1; }
test "$(uname -m)" = aarch64 || { echo 'Use the RK3568 64-bit Linux board'; exit 1; }
version=$(cat /sys/module/adas_video_pcie/version 2>/dev/null || true)
test "$version" = 5.2-marker-ack || { echo 'First run: sudo sh install_driver.sh'; exit 1; }
case "${1:-}" in
 passthrough|yolo_only)
  export ADAS_RUNTIME_MODE=$1
  unset ADAS_HUD_MODE
  shift
  exec sh "$root/rk3568/start_yolo.sh" --mode "$ADAS_RUNTIME_MODE" --preprocess cpu "$@"
  ;;
esac
if [ ! -x "$root/launcher/return_input" ]; then
 cc -O2 -std=c11 -Wall -Wextra -Werror -I "$root/rk3568/npu_runtime" \
  "$root/launcher/tools/return_input.c" -lX11 -lXi -o "$root/launcher/return_input"
fi
test -c /dev/adas_video0 || { echo 'Prepare matching FPGA/PCIe first: sh /opt/adas-pcie/current/start_adas_pcie.sh'; exit 1; }
export DISPLAY=${DISPLAY:-:0} QT_QPA_PLATFORM=xcb QT_XCB_GL_INTEGRATION=none
if [ -z "${XAUTHORITY:-}" ] && [ -f /var/run/lightdm/root/:0 ]; then export XAUTHORITY=/var/run/lightdm/root/:0; fi
export ADAS_RK_SCRIPTS="$root/rk3568"
mkdir -p "$root/launcher/_tmp"
exec sh "$root/launcher/run_launcher.sh" --run-mode script "$@"
