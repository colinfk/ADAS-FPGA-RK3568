#ifndef ADAS_INTERSECTION_H
#define ADAS_INTERSECTION_H
#include "adas_state.h"
#include "key_force_drive.h"

enum intersection_command { JUNCTION_NONE, JUNCTION_STOP, JUNCTION_LEFT, JUNCTION_STRAIGHT, JUNCTION_RIGHT };
enum intersection_permission { JUNCTION_BLOCKED, JUNCTION_UNCONFIRMED, JUNCTION_ALLOWED };
struct intersection_keys {
    uint32_t left,right,sequence;
    double last,pressed;
    unsigned pending,down;
    int initialized,chord,stopped,ignore;
};
struct intersection_view {
    int enabled,command,motion,red_alarm,pedestrian_alarm,stale,pedestrian_present;
    enum intersection_permission permission[3];
};
struct intersection_state {
    KeyDrive drive;
    struct detection subjects[16];
    int subject_count;
    double origin,last,last_visible;
    int initialized;
};
/* Coherent BAR read; 1 valid, 0 transient, -1 missing ABI. */
int intersection_key_snapshot(const volatile uint32_t *map,uint32_t *sequence,
    uint32_t *left,uint32_t *right,unsigned *down);
/* -1 session invalidated; 0 no command; 1..4 command. */
int intersection_key_step(struct intersection_keys *state,uint32_t sequence,
    uint32_t left,uint32_t right,unsigned down,double now);
void intersection_update(struct intersection_state *state,int command,
    const struct detection *objects,int count,const struct lane lanes[2],
    const struct adas_status *status,double now,struct intersection_view *view);
void intersection_present(const struct intersection_view *view,struct adas_status *status);
#endif
