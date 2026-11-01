#ifndef ADAS_TRACKER_H
#define ADAS_TRACKER_H
#include "adas_core.h"
struct adas_tracker;
struct adas_box_display;
struct adas_box_display *adas_box_display_create(void);
void adas_box_display_destroy(struct adas_box_display *state);
int adas_box_display_update(struct adas_box_display *state,const uint8_t *image,
    uint32_t source,double arrived,int quad,const struct detection *raw,int count,
    struct detection shown[MAX_DETECTIONS],int raw_indices[MAX_DETECTIONS]);
struct track_view {
    uint64_t id;
    unsigned hits,confirmed;
    float vx,vy;
};
struct adas_tracker *adas_tracker_create(void);
void adas_tracker_destroy(struct adas_tracker *tracker);
/* Lightweight IoU+EMA telemetry; velocity is measured center displacement per second.
 * Output order/count equal raw input; no boxes are emitted for missed objects.
 * Returns 1 on temporal/scene reset, 0 otherwise, -1 on invalid input. */
int adas_tracker_update(struct adas_tracker *tracker,const uint8_t *image,
    uint32_t source_id,double arrived,const struct detection *raw,int count,
    struct detection shown[MAX_DETECTIONS],struct track_view views[MAX_DETECTIONS]);
#endif
