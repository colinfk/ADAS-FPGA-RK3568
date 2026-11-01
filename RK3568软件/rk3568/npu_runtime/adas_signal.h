#ifndef ADAS_SIGNAL_H
#define ADAS_SIGNAL_H
#include "adas_state.h"
struct light_track {
    int valid, acquired;
    enum adas_light color;
    float x0,y0,x1,y1;
    enum adas_light pending;
    unsigned pending_run,dwell,hold;
    uint32_t last_fid;
    int selected;
    int allow_switch;
    unsigned challenge_run;
};
void adas_light_track_reset(struct light_track *t);
int adas_light_track_step(struct light_track *t,const struct detection *boxes,const enum adas_light *colors,
    const int *red,const int *green,const float *scores,int count,int fallback,int reset,uint32_t fid,
    enum adas_light *shown_color,struct candidate *shown_box);
struct signal_roi { int enabled,x,y,w,h,intersection; };
enum signal_roi_mode { SIGNAL_ROI_RULES=1, SIGNAL_ROI_INT8=2 };
struct signal_detector {
    struct adas_signal pending;
    unsigned hits;
    double since,last_time;
    uint32_t last_source;
    uint8_t *mask;
    int *queue;
};
enum adas_light pixel_light_color(const uint8_t *image,const struct detection *box,int *red_votes,int *green_votes);
int adas_signal_init(struct signal_detector *detector);
void adas_signal_close(struct signal_detector *detector);
void light_color_max_infer(const uint8_t pixels[32*32*3],int32_t logits[4]);
int light_color_max_input(const uint8_t *image,int x0,int y0,int x1,int y1,uint8_t pixels[32*32*3]);
void adas_signal_detect(struct signal_detector *detector,const uint8_t *image,const struct features *features,
    const struct detection *objects,int count,const struct signal_roi *roi,double arrived,struct adas_signal *result);
#endif
