#ifndef ADAS_NAV_H
#define ADAS_NAV_H
#include "adas_core.h"
#include <stdio.h>

#define ADAS_NAV_ROAD_X 594
#define ADAS_NAV_ROAD_Y 134
#define ADAS_NAV_ROAD_W 1326
#define ADAS_NAV_ROAD_H 870
enum adas_nav_direction { NAV_UNKNOWN, NAV_LEFT, NAV_RIGHT, NAV_STRAIGHT };
struct adas_nav_observation {
    enum adas_nav_direction direction;
    int valid,distance_m,near_turn;
    float confidence,direction_confidence,distance_confidence;
};
/* Capture owns the observation until publication; the post worker alone owns
 * debounce state. No frame, glyph or route state is shared between slots. */
struct adas_nav_slot { uint8_t *original; struct adas_nav_observation observation; };
struct adas_nav_state {
    struct adas_nav_observation route;
    enum adas_nav_direction pending;
    unsigned hits;
    int pending_distance,have_frame;
    uint32_t source;
    double arrived;
    enum adas_nav_direction ar_direction;
    double ar_turn_until;
    unsigned char *signal_mask;
    int *signal_queue;
    int signal_x,signal_y,signal_color,signal_hits,signal_have;
    double signal_first,signal_seen,signal_frame;
    enum adas_nav_direction signal_direction;
};
struct adas_signal;
enum adas_nav_direction adas_nav_current_direction(const struct adas_nav_observation *route);
int adas_nav_current_control(const struct adas_nav_observation *route);
void adas_nav_close(struct adas_nav_state *state);
void adas_nav_pixel_signal(struct adas_nav_state *state,const uint8_t *road,
    double now,struct adas_signal *out);
void adas_nav_pixel_signal_with_objects(struct adas_nav_state *state,const uint8_t *road,
    const struct detection *objects,int count,double now,struct adas_signal *out);
int adas_nav_slot_init(struct adas_nav_slot *slot);
void adas_nav_slot_close(struct adas_nav_slot *slot);
int adas_nav_parse_rgb565(const uint8_t *image,struct adas_nav_observation *out);
int adas_nav_capture(struct adas_nav_slot *slot,uint8_t *road,struct features *features);
void adas_nav_compose(struct adas_nav_slot *slot,const uint8_t *road);
void adas_nav_update(struct adas_nav_state *state,const struct adas_nav_observation *observation,
    uint32_t source,double arrived,int reset);
const char *adas_nav_direction_name(enum adas_nav_direction direction);
/* Existing controls use left=-1, straight=0, right=1, unknown=2. */
int adas_nav_control_route(const struct adas_nav_observation *route);
void adas_nav_write_json(FILE *file,const struct adas_nav_observation *pixel,
    const struct adas_nav_observation *route);
#endif
