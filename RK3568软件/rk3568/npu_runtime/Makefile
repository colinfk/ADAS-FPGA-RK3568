CC ?= cc
CFLAGS ?= -O2 -g
WARN = -std=c11 -Wall -Wextra -Werror -Wformat=2 -Wshadow
SOURCES = play_adas_yolo.c adas_core.c npu_runtime.c adas_state.c adas_signal.c adas_hud.c adas_nav.c adas_intersection.c key_force_drive.c preprocess.c adas_tracker.c lane_cnn_v2.c lane_cnn_cpu.c lane_pipeline.c
.PHONY: all clean test
all: play_adas_yolo check_preprocess
play_adas_yolo: $(SOURCES) adas_nav.h nav_glyphs.h nav_signal_templates.h adas_intersection.h key_force_drive.h adas_core.h npu_runtime.h adas_state.h adas_signal.h adas_hud.h hud_font.h cruise_font.h light_params_v2.h preprocess.h adas_tracker.h lane_cnn_v2.h lane_cnn_cpu.h lane_roi_config.h lane_cnn_params_v2.h lane_pipeline.h vendor/rknn_api.h $(wildcard vendor/librga/include/*.h)
	$(CC) $(CFLAGS) $(WARN) $(SOURCES) -pthread -ldl -lm -o $@
check_preprocess: check_preprocess.c preprocess.c preprocess.h adas_core.c $(wildcard vendor/librga/include/*.h)
	$(CC) $(CFLAGS) $(WARN) check_preprocess.c preprocess.c adas_core.c -ldl -lm -o $@
test:
	python3 tests/test_runtime.py
	python3 tests/test_rga.py
	python3 tests/test_cnn_runtime.py
	python3 tests/test_presentation.py
	python3 tests/test_road_stage.py
	python3 tests/test_competition.py
	python3 tests/test_hud_chinese.py
	python3 ../../tools/test_cruise_ui.py --quick
clean:
	rm -f play_adas_yolo check_preprocess

# Separate intersection experiment; common capture, NPU and lane sources stay shared.
TRAFFIC10_SOURCES = traffic10/play_adas_yolo.c adas_core.c npu_runtime.c adas_state.c adas_signal.c traffic10/adas_hud.c adas_nav.c traffic10/adas_intersection.c key_force_drive.c preprocess.c adas_tracker.c lane_cnn_v2.c lane_cnn_cpu.c lane_pipeline.c
bin/play_traffic10_aarch64: $(TRAFFIC10_SOURCES) traffic10/adas_hud.h traffic10/adas_intersection.h
	$(CC) $(CFLAGS) $(WARN) -I. $(TRAFFIC10_SOURCES) -pthread -ldl -lm -o $@
