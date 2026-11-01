#!/bin/sh
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
test "$(uname -m)" = aarch64 || { echo 'Build on aarch64 RK Linux'; exit 1; }
command -v cc >/dev/null && command -v make >/dev/null
cd "$root/rk3568/npu_runtime"
make -B play_adas_yolo
mkdir -p bin
cp play_adas_yolo bin/play_adas_yolo_aarch64
cc -O2 -std=c11 -Wall -Wextra -Werror -I "$root/rk3568/npu_runtime" \
 "$root/launcher/tools/return_input.c" -lX11 -lXi -o "$root/launcher/return_input"
echo 'RK application and return helper built'
