#ifndef ADAS_HUD_H
#define ADAS_HUD_H
#include "adas_state.h"
#include "adas_tracker.h"
#include "adas_nav.h"
#include "adas_intersection.h"

#define ADAS_HUD_RETURN_X 1720
#define ADAS_HUD_RETURN_Y 988
#define ADAS_HUD_RETURN_W 176
#define ADAS_HUD_RETURN_H 72
void adas_hud_return_button(uint8_t *image);
void adas_hud_yolo_only(uint8_t *image,const struct detection *objects,int count,uint32_t frame_id);
void adas_hud_intersection(uint8_t *image,const struct features *features,
    const struct detection *objects,int count,const struct lane lanes[2],
    const struct adas_status *status,const struct intersection_view *view,int debug,const char *timing);
enum adas_nav_action { NAV_WAIT, NAV_GUIDE, NAV_CAUTION, NAV_SLOW_NEAR, NAV_STOP_RED, NAV_STOP_HAZARD };
/* NEAR/APPROACH is an amber following-gap advisory in NAV. Only independently
 * confirmed red or other stop hazards may select a red stop presentation. */
/* NAV-only policy: an ROI is directional only after explicit installation
 * binding. SEARCH is never promoted, including a stable red candidate. */
void adas_hud_nav_bind_signal(struct adas_signal *signal,const struct adas_nav_observation *route,
    enum adas_nav_direction roi_direction);
enum adas_nav_action adas_hud_nav_action(const struct adas_nav_observation *route,const struct adas_status *status);
const char *adas_hud_nav_reason(const struct adas_nav_observation *route,const struct adas_status *status);
void adas_hud_nav_road(uint8_t *image,const struct adas_nav_observation *route,
    const struct lane lanes[2],const struct adas_status *status,const struct detection *objects,int count);
void adas_hud_nav_card(uint8_t *image,const struct adas_nav_observation *route,const struct adas_status *status);
void adas_hud_nav_reference(uint8_t *image,const struct adas_nav_observation *route,
    enum adas_nav_direction direction,const struct adas_status *status,const struct detection *objects,int count);
enum hud_mode { HUD_MODE_CRUISE, HUD_MODE_INTERSECTION, HUD_MODE_NAV, HUD_MODE_SURROUND };
void adas_hud_set_mode(enum hud_mode mode);
void adas_hud_set_pixel_light(int on);
void adas_hud_cnn(uint8_t *image,const struct adas_status *status,const char *sync,int fresh);
void adas_hud_pixel_light_mark(uint8_t *image,int x0,int y0,int x1,int y1,enum adas_light light,float confidence,int intersection);
const char *adas_hud_event_name(const struct adas_status *status);
const char *adas_hud_lane_value_label(const struct adas_status *status,int side);
int adas_hud_intersection_snapshot(const struct adas_status *status,const struct intersection_view *view,int quad,char *json,size_t capacity);
int adas_hud_snapshot(const struct adas_status *status,int quad,char *json,size_t capacity);
void adas_hud(uint8_t *image,const struct features *features,const struct detection *detections,
              int count,const struct lane lanes[2],const struct adas_status *status,
              int debug,const char *timing);
void adas_hud_tracked(uint8_t *image,const struct features *features,const struct detection *detections,
              int count,const struct lane lanes[2],const struct adas_status *status,
              int debug,const char *timing,const struct track_view *tracks);
#endif
