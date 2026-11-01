#ifndef LANE_PIPELINE_H
#define LANE_PIPELINE_H
#include "adas_state.h"
#include "lane_cnn_v2.h"
/* Modes: 0 rules, 1 fixed FPGA CNN, 2 aligned FPGA or current-frame CPU ROI. */
struct lane_pipeline {
    struct lane_history classes[2];
    struct lane_geometry_history geometry[2];
    struct lane_cnn_display display[2];
    enum lane_source previous_source[2];
};
struct lane_observation {
    struct lane lanes[2],fit[2];
    enum lane_source source[2];
    int roi_hits[2],guided[2],top[2];
    unsigned applied;
    int gap_veto[2];
    unsigned raw_class[2],raw_confidence[2];
    int continuity_guard[2];
};
void lane_pipeline_step(struct lane_pipeline *p,const uint8_t *image,
    const struct features *f,const struct detection *d,int n,int mode,
    const struct lane_cnn_snapshot *cnn,double now,int reset,struct lane_observation *out);
void lane_pipeline_status(const struct lane_pipeline *p,const struct lane_observation *out,
    const struct lane_cnn_snapshot *cnn,int mode,double now,struct adas_status *status);
#endif
