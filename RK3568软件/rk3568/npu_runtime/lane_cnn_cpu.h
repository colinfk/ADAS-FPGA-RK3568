#ifndef LANE_CNN_CPU_H
#define LANE_CNN_CPU_H
#include "adas_core.h"
int lane_cnn_quad_input(const uint8_t *image,const struct lane *fit,uint8_t gray[64*32]);
struct lane_cpu_result { unsigned valid,cls,margin; int32_t logits[3]; };
int lane_cnn_guided_input(const uint8_t *image,const struct lane *fit,uint8_t gray[64*32]);
int lane_cnn_guided_top(const struct lane *fit);
void lane_cnn_cpu_infer(const uint8_t gray[64*32],struct lane_cpu_result *out);
/* Fraction of full-height fixed-ROI rows containing the observed fitted line. */
int lane_cnn_roi_hits(const struct lane *fit,int side);
int lane_cnn_roi_aligned(const struct lane *fit,int side);
int lane_cnn_roi_hits_version(const struct lane *fit,int side,unsigned version);
int lane_cnn_roi_aligned_version(const struct lane *fit,int side,unsigned version);
/* Bit-exact ROI v4 reference for offline RTL/input comparison. */
int lane_cnn_roi4_input(const uint8_t *image,int side,uint8_t gray[64*32]);
#endif
