#!/bin/sh
set -eu
root=$(CDPATH= cd -- "$(dirname -- "$0")/.." && pwd)
cd "$root"
${CC:-cc} -O2 -g -std=c11 -Wall -Wextra -Werror -Wformat=2 -Wshadow \
    -Itraffic10 -I. traffic10/final_source/play_adas_yolo.c \
    traffic10/final_source/adas_core.c npu_runtime.c adas_state.c adas_signal.c \
    traffic10/adas_hud.c adas_nav.c traffic10/adas_intersection.c key_force_drive.c \
    preprocess.c adas_tracker.c lane_cnn_v2.c lane_cnn_cpu.c lane_pipeline.c \
    -pthread -ldl -lm -o bin/play_traffic10_rebuilt_aarch64
