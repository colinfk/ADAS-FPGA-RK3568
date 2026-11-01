#ifndef LANE_ROI_CONFIG_H
#define LANE_ROI_CONFIG_H
#include "adas_core.h"
#include <math.h>
static inline int perspective_row_x0(int side,int y) {
    int remain=1023-y,shift=(remain>>1)+(remain>>3);
    return side?1312-shift:480+shift;
}
static inline int lane_roi_hits_version(const struct lane *fit,int side,unsigned version) {
    if(!fit||!fit->valid||side<0||side>1||!isfinite(fit->slope)||!isfinite(fit->intercept)||
       (version!=3&&version!=4))return -1;
    int perspective=version==4&&fabsf(fit->slope)>=.05f,hits=0;
    for(int row=perspective?24:0;row<64;row++) {
        int y=perspective?516+8*row:770+4*row;
        float left=perspective?(float)perspective_row_x0(side,y):side?1312.f:480.f;
        float x=fit->slope*y+fit->intercept;
        if(x>=left&&x<left+128)hits++;
    }
    return hits;
}
static inline int lane_roi_aligned_version(const struct lane *fit,int side,unsigned version) {
    int perspective=fit&&version==4&&fabsf(fit->slope)>=.05f;
    if(lane_roi_hits_version(fit,side,version)<(perspective?26:52))return 0;
    int top=perspective?864:768;
    for(int y=top;y<=1023;y+=1023-top) {
        float left=perspective?(float)perspective_row_x0(side,y):side?1312.f:480.f;
        float x=fit->slope*y+fit->intercept;
        if(x<left+4||x>=left+124)return 0;
    }
    return 1;
}
#endif
