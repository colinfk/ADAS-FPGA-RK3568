#ifndef ADAS_CORE_H
#define ADAS_CORE_H
#include <stddef.h>
#include <stdint.h>
#define ADAS_W 1920
#define ADAS_H 1080
#define IMAGE_BYTES (ADAS_W*ADAS_H*2u)
#define PAYLOAD_BYTES (IMAGE_BYTES+256u)
#define ALLOC_BYTES 4149248u
#define MAX_DETECTIONS 300
/* Software-only opt-in, set after the physical trailer has passed validation. */
#define ADAS_SOURCE_QUAD (1u<<30)
#define ADAS_SOURCE_STABLE_HUD (1u<<29)
#define ADAS_SOURCE_QUAD_QUARTER (1u<<28) /* Software layout tag, never a DMA ABI bit. */
enum quad_layout { QUAD_LAYOUT_LEGACY, QUAD_LAYOUT_REAR_QUARTER };
#define QUAD_TOP_Y 84
#define QUAD_MOD_H 256
#define QUAD_FRONT_Y0 (QUAD_TOP_Y+QUAD_MOD_H)
#define QUAD_FRONT_END ADAS_H
#define QUAD_ROAD_MIN 512
#define QUAD_ROAD_MAX 1023
#define QUAD_LEFT_SEAM 640
#define QUAD_RIGHT_SEAM 1280
static inline int adas_quad_front_y(uint32_t kind) { return kind&ADAS_SOURCE_QUAD_QUARTER?270:QUAD_FRONT_Y0; }
static inline int adas_quad_rear_y(uint32_t kind) { return kind&ADAS_SOURCE_QUAD_QUARTER?0:QUAD_TOP_Y; }
struct candidate { unsigned valid,x0,y0,x1,y1,cx,cy,rows; };
struct features {
    uint32_t source_id,layout,mode,profile,source_kind,checksum,road[20];
    struct candidate candidates[3];
};
struct detection { float x0,y0,x1,y1,score; int cls; };
enum lane_type { LANE_UNKNOWN, LANE_SOLID, LANE_DASHED };
/* Display classes follow the FPGA encoding; UNKNOWN means no usable observation. */
enum lane_class { LANE_CLASS_NONE, LANE_CLASS_DASHED, LANE_CLASS_SOLID, LANE_CLASS_INVALID, LANE_CLASS_UNKNOWN };
static inline enum lane_type adas_lane_type(enum lane_class cls) {
    return cls==LANE_CLASS_DASHED?LANE_DASHED:cls==LANE_CLASS_SOLID?LANE_SOLID:LANE_UNKNOWN;
}
static inline enum lane_class adas_lane_class(enum lane_type type) {
    return type==LANE_DASHED?LANE_CLASS_DASHED:type==LANE_SOLID?LANE_CLASS_SOLID:LANE_CLASS_UNKNOWN;
}
const char *adas_lane_class_name(enum lane_class cls);
struct lane { float slope,intercept,coverage,residual; unsigned valid; enum lane_type type; };
struct lane_history {
    enum lane_type stable,pending;
    unsigned hits;
    unsigned hold_miss; /* frames without confirmation, including geometry jumps */
    struct lane last;
};
/* Current fit geometry has independent confirmation and never holds a missing fit. */
struct lane_geometry_history { struct lane last; unsigned hits; };
void adas_lane_geometry_stabilize(struct lane_geometry_history *history,struct lane *lane,int reset);
uint32_t adas_xor(const uint8_t *image);
uint32_t adas_packet_checksum(const uint8_t *payload,unsigned long captured,int fast);
int adas_parse(const uint8_t *payload,struct features *f,uint32_t checksum);
void adas_preprocess(const uint8_t *image,uint8_t *rgb);
void adas_preprocess_signal_zoom(const uint8_t *image,uint8_t *rgb);
void adas_preprocess_signal_wide(const uint8_t *image,uint8_t *rgb);
int adas_quad_preprocess(const uint8_t *image,int region,uint8_t *rgb);
int adas_quad_preprocess_layout(const uint8_t *image,int region,enum quad_layout layout,uint8_t *rgb);
int adas_quad_decode(const float *outputs[9],int region,float confidence,float iou,
                     struct detection detections[MAX_DETECTIONS]);
int adas_quad_decode_layout(const float *outputs[9],int region,enum quad_layout layout,
    float confidence,float iou,struct detection detections[MAX_DETECTIONS]);
int adas_quad_append(struct detection all[MAX_DETECTIONS],int count,
                     const struct detection *part,int size);
void adas_quad_features(const uint8_t *image,struct features *f);
void adas_lane_quad_masked(const uint8_t *image,int side,
    const struct detection *objects,int count,struct lane *lane);
void adas_lane_reject_hood(const uint8_t *image,const struct detection *objects,int count,struct lane *lane);
/* Outputs must be per-tensor dequantized FP32 NCHW, ordered reg/cls/sum. */
int adas_decode(const float *outputs[9],float confidence,float iou,
                struct detection detections[MAX_DETECTIONS]);
int adas_decode10(const float *outputs[9],float confidence,float iou,struct detection detections[MAX_DETECTIONS]);
void adas_lane(const uint8_t *image,const struct candidate *candidate,int side,struct lane *lane);
int adas_crosswalk_confirm(const uint8_t *image,const struct candidate *candidate);
void adas_lane_masked(const uint8_t *image,const struct candidate *candidate,int side,
                      const struct detection *objects,int count,struct lane *lane);
void adas_lane_stabilize(struct lane_history *history,struct lane *lane,int reset);
const char *adas_class_name(int cls);
#endif
