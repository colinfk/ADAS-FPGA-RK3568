#ifndef ADAS_STATE_H
#define ADAS_STATE_H
#include "adas_core.h"
enum adas_level { ADAS_MONITOR, ADAS_CAUTION, ADAS_DANGER };
enum adas_turn_feedback { TURN_IDLE, TURN_WAIT, TURN_RED, TURN_GREEN };
enum adas_hint { HINT_UNKNOWN, HINT_SOLID, HINT_DASHED, HINT_OCCUPIED };
enum adas_light { LIGHT_UNKNOWN, LIGHT_RED, LIGHT_GREEN, LIGHT_YELLOW };
enum signal_source { SIGNAL_SEARCH, SIGNAL_ROI, SIGNAL_MANUAL };
enum lane_source { LANE_SOURCE_RULES, LANE_SOURCE_FPGA, LANE_SOURCE_CPU_ROI, LANE_SOURCE_WAIT };
enum road_profile { ROAD_AUTO, ROAD_URBAN, ROAD_HIGHWAY }; /* configured context, not a map inference */
enum road_scene { ROAD_UNCERTAIN, ROAD_LANE_MONITOR, ROAD_JUNCTION_CANDIDATE, ROAD_DENSE_TRAFFIC,
                  ROAD_VRU_WATCH, ROAD_LOW_VISIBILITY, ROAD_CUT_IN_WATCH };
enum road_action { ROAD_OBSERVE, ROAD_CHECK_INPUT, ROAD_CHECK_LANE, ROAD_WATCH_SIGNAL,
                   ROAD_PREPARE_STOP, ROAD_KEEP_STOPPED, ROAD_SLOW_FOR_VRU, ROAD_INCREASE_GAP,
                   ROAD_KEEP_CENTER, ROAD_SLOW_VISIBILITY, ROAD_WATCH_CROSSING, ROAD_NO_LANE_CHANGE, ROAD_YELLOW_WATCH };
struct adas_signal {
    enum adas_light candidate;
    enum signal_source source;
    struct candidate box;
    float confidence;
    int stable;
};
enum adas_event {
    EVENT_NEAR=1u, EVENT_PEDESTRIAN=2u, EVENT_APPROACH=4u,
    EVENT_EDGE_LEFT=8u, EVENT_EDGE_RIGHT=16u, EVENT_SOLID_FORCE=32u,
    EVENT_RED_MOVING=64u, EVENT_OVERSPEED=128u, EVENT_ZEBRA=256u,
    EVENT_DARK=512u, EVENT_STALE=1024u, EVENT_NO_LANE=2048u,
    EVENT_TURN_BLOCKED=4096u, EVENT_RED_STOP=8192u, EVENT_CUT_IN=16384u, EVENT_REAR_NEAR=32768u
};
enum adas_control_source { ADAS_CONTROL_MONITOR, ADAS_CONTROL_SIMULATED, ADAS_CONTROL_PHYSICAL };
struct adas_controls {
    int enabled,turn,moving,motion_known,speed,speed_known,speed_limit,route;
    enum adas_light light;
    int forced_turn;
    double forced_until;
    enum road_profile road_profile;
    int ego_lane,auto_signal_fsm; /* Explicit interactive demonstration controls. */
};
struct hud_view {
    int enabled,ready,side_valid[2],occupied[2],force_from_hold,turn_blocked;
    enum lane_type type[2];
    enum adas_hint hints[2];
    uint32_t events;
    enum adas_level level;
};
struct adas_status {
    enum adas_level level;
    uint32_t events,source_id;
    enum adas_hint hints[2];
    enum lane_class display_class[2];
    int display_cnn,display_held[2],occupied[2];
    uint32_t display_frame[2];
    double display_age_ms[2];
    int lane_geometry,lead_index,turn,route,moving,motion_known,speed,speed_known,speed_limit,simulated;
    enum adas_light light; /* FSM: ROI-stable (+manual); drives EVENT_RED_* */
    enum adas_light hud_light; /* Presentation: also shows stable SEARCH candidate */
    struct adas_signal signal;
    float corridor_left,corridor_right,dark_ratio,gray_mean;
    double age_ms,phase_time;
    int road_ready,vehicle_count,vru_count;
    enum road_profile road_profile;
    enum road_scene road_scene;
    enum road_action road_action;
    float lane_center_norm,approach_rate;
    int approach_known;
    enum lane_source lane_source[2];
    int roi_hits[2],roi_guided[2],roi_top[2];
    float roi_slope[2],roi_intercept[2];
    uint32_t cnn_roi_version,cnn_model_version;
    int cut_in_side; /* image trajectory candidate, not a physical intent claim */
    int ego_lane,auto_signal_fsm;
    int coasting_valid[2]; /* HUD only: never valid geometry or an FSM FORCE input. */
    float coasting_slope[2],coasting_intercept[2];
    enum lane_type coasting_type[2];
    uint32_t fpga_edge_pixels,fpga_morph_pixels,fpga_white_left,fpga_white_right;
    int fpga_zebra_candidate;
    int gap_veto[2]; /* Long-fit correction, distinct from raw CNN output. */
    struct hud_view hud; /* Presentation only; never a geometry or FSM input. */
    int rear_hud_occupied[2];
};
struct adas_state {
    enum adas_level level,pending;
    uint32_t held_events,last_source;
    unsigned hits;
    double pending_since,hold_until,last_arrived;
    struct detection lead;
    int have_lead,have_frame;
    struct detection previous_objects[MAX_DETECTIONS];
    struct lane previous_lanes[2];
    int previous_count,previous_geometry,cut_in_side;
    double cut_in_until;
    struct lane coast_lanes[2];
    unsigned coast_miss[2];
    unsigned occ_hold[2],hud_age,hud_hits,type_hits[2],approach_hits;
    int disp_occupied[2],hud_have,hud_turn,hud_ego,hud_moving;
    uint32_t hud_event,hud_pending;
    enum lane_type hud_type[2],type_pending[2];
};
void adas_hud_prepare(struct adas_state *state,const struct features *features,
    const struct lane lanes[2],const struct adas_controls *controls,int object_count,
    struct adas_status *status);
enum adas_turn_feedback adas_turn_feedback(const struct adas_status *status,const struct lane lanes[2]);
void adas_controls_init(struct adas_controls *c,int enabled);
void adas_control_key(struct adas_controls *c,int key,double now);
void adas_state_update(struct adas_state *state,const struct features *features,
    const struct detection *detections,int count,const struct lane lanes[2],
    const struct adas_controls *controls,const struct adas_signal *signal,
    double arrived,double now,struct adas_status *status);
void adas_state_update_with_rear(struct adas_state *state,const struct features *features,
    const struct detection *detections,int count,const struct lane lanes[2],
    const struct adas_controls *controls,const struct adas_signal *signal,
    double arrived,double now,int rear_left,int rear_right,struct adas_status *status);
const char *adas_level_name(enum adas_level level);
const char *adas_hint_name(enum adas_hint hint);
const char *adas_road_scene_name(enum road_scene scene);
const char *adas_road_action_name(enum road_action action);
const char *adas_lane_source_name(enum lane_source source);
#endif
