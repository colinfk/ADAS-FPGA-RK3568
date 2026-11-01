#include "adas_state.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
enum adas_turn_feedback adas_turn_feedback(const struct adas_status *s,const struct lane lanes[2]) {
    if(!s->turn)return TURN_IDLE;
    int side=s->turn>0;
    if(!lanes[side].valid||!isfinite(lanes[side].slope)||!isfinite(lanes[side].intercept)||
       (s->events&(EVENT_STALE|EVENT_DARK)))return TURN_WAIT;
    if(lanes[side].type==LANE_SOLID)return TURN_RED;
    if(lanes[side].type==LANE_DASHED&&!s->occupied[side]&&!s->rear_hud_occupied[side]&&
       !(s->hud.enabled&&s->hud.occupied[side]))return TURN_GREEN;
    return TURN_WAIT;
}
void adas_controls_init(struct adas_controls *c,int enabled) {
    memset(c,0,sizeof(*c)); c->enabled=enabled; c->speed_limit=50; c->route=2;
}
void adas_control_key(struct adas_controls *c,int key,double now) {
    if(c->enabled!=ADAS_CONTROL_SIMULATED) return;
    if(key>='A'&&key<='Z') key+=32;
    switch(key) {
    case 'a': c->turn=-1; break;
    case 'd': c->turn=1; break;
    case 'c': c->turn=0; c->forced_turn=0; c->forced_until=0; break;
    case 'q': c->turn=-1; c->forced_turn=-1; c->forced_until=now+1.; break;
    case 'e': c->turn=1; c->forced_turn=1; c->forced_until=now+1.; break;
    case 'w': c->moving=1; c->motion_known=1; break;
    case 's': c->moving=0; c->motion_known=1; c->speed=0; c->speed_known=1; break;
    case 'r': c->light=LIGHT_RED; break;
    case 'g': c->light=LIGHT_GREEN; break;
    case 'y': c->light=LIGHT_YELLOW; break;
    case 'u': c->light=LIGHT_UNKNOWN; break;
    case 'j': c->ego_lane=-1; c->route=-1; break;
    case 'k': c->ego_lane=0; c->route=0; break;
    case 'l': c->ego_lane=1; c->route=1; break;
    case 'm': c->auto_signal_fsm=!c->auto_signal_fsm; break;
    case '+': case '=': c->speed_known=1; if(c->speed<150) c->speed+=5; c->moving=c->speed>0; c->motion_known=1; break;
    case '-': c->speed_known=1; if(c->speed>0) c->speed-=5; c->moving=c->speed>0; c->motion_known=1; break;
    case '1': c->route=-1; break;
    case '2': c->route=0; break;
    case '3': c->route=1; break;
    case '0': { int limit=c->speed_limit;enum road_profile profile=c->road_profile; adas_controls_init(c,1); c->speed_limit=limit;c->road_profile=profile; break; }
    }
}
const char *adas_level_name(enum adas_level l) { return l==ADAS_DANGER?"DANGER":l==ADAS_CAUTION?"CAUTION":"MONITOR"; }
const char *adas_hint_name(enum adas_hint h) {
    return h==HINT_SOLID?"SOLID_HOLD":h==HINT_DASHED?"DASHED_CHECK_REAR":h==HINT_OCCUPIED?"OCCUPIED_WAIT":"UNKNOWN";
}
const char *adas_lane_source_name(enum lane_source source) {
    return source==LANE_SOURCE_FPGA?"FPGA_ROI":source==LANE_SOURCE_CPU_ROI?"CPU_GUIDED_ROI":
        source==LANE_SOURCE_WAIT?"WAIT":"RULES";
}
const char *adas_road_scene_name(enum road_scene scene) {
    static const char *const names[]={"UNCERTAIN","LANE MONITOR","JUNCTION CANDIDATE","DENSE TRAFFIC","VRU WATCH","LOW VISIBILITY","CUT IN CANDIDATE"};
    return scene>=ROAD_UNCERTAIN&&scene<=ROAD_CUT_IN_WATCH?names[scene]:names[0];
}
const char *adas_road_action_name(enum road_action action) {
    static const char *const names[]={"KEEP OBSERVING","CHECK INPUT / DRIVER ATTENTION","VERIFY LANE POSITION",
        "VERIFY TRAFFIC SIGNAL","PREPARE TO STOP","KEEP STOPPED","SLOW / WATCH ROAD USERS",
        "INCREASE FOLLOWING GAP","KEEP CENTER / WATCH EDGE","SLOW / LOW VISIBILITY",
        "WATCH CROSSING","AVOID LANE CHANGE","YELLOW / WATCH SIGNAL"};
    return action>=ROAD_OBSERVE&&action<=ROAD_YELLOW_WATCH?names[action]:names[0];
}
static float at(const struct lane *l,float y) { return l->slope*y+l->intercept; }
static int geometry(const struct lane lanes[2],int quad) {
    int top=720,bottom=quad?QUAD_ROAD_MAX:1076;
    if(!lanes[0].valid||!lanes[1].valid) return 0;
    if(lanes[0].slope>=-.1f||lanes[1].slope<=.1f) return 0;
    float top_width=at(&lanes[1],top)-at(&lanes[0],top);
    float bottom_width=at(&lanes[1],bottom)-at(&lanes[0],bottom);
    float horizon=(lanes[0].intercept-lanes[1].intercept)/(lanes[1].slope-lanes[0].slope);
    float vanishing_x=at(&lanes[0],horizon);
    if(bottom_width<top_width+100||horizon<280||horizon>760||vanishing_x<480||vanishing_x>1440) return 0;
    for(int y=top;y<=(quad?QUAD_ROAD_MAX:1080);y+=(quad?188:360)) {
        float left=at(&lanes[0],y),right=at(&lanes[1],y),width=right-left;
        if(!isfinite(left)||!isfinite(right)||left<0||right>ADAS_W||width<140||width>1700) return 0;
    }
    return 1;
}
static float fraction(float lo,float hi,float x0,float x1) {
    return fmaxf(0,fminf(hi,x1)-fmaxf(lo,x0))/fmaxf(x1-x0,1);
}
static float iou(const struct detection *a,const struct detection *b) {
    float area=fmaxf(0,fminf(a->x1,b->x1)-fmaxf(a->x0,b->x0))*fmaxf(0,fminf(a->y1,b->y1)-fmaxf(a->y0,b->y0));
    return area/fmaxf((a->x1-a->x0)*(a->y1-a->y0)+(b->x1-b->x0)*(b->y1-b->y0)-area,1);
}
void adas_state_update_with_rear(struct adas_state *s,const struct features *f,const struct detection *d,int n,
    const struct lane lanes[2],const struct adas_controls *c,const struct adas_signal *signal,
    double arrived,double now,int rear_left,int rear_right,struct adas_status *out) {
    int quad=(f->source_kind&ADAS_SOURCE_QUAD)!=0;
    float front_height=quad?QUAD_FRONT_END:ADAS_H;
    int front_y=adas_quad_front_y(f->source_kind),rear_y=adas_quad_rear_y(f->source_kind);
    float front_top=720;
    struct detection front[MAX_DETECTIONS];int indices[MAX_DETECTIONS],front_count=0,rear_near=0;
    int rear_hud[2]={0};
    uint32_t display_delta=f->source_id-s->last_source;
    int display_contiguous=s->have_frame&&arrived>s->last_arrived&&arrived-s->last_arrived<=.5&&display_delta&&display_delta<=15;
    if(quad) {
        rear_left=rear_right=0;
        for(int i=0;i<n&&i<MAX_DETECTIONS;i++) {
            const struct detection *v=&d[i];
            if(v->y0>=front_y&&v->y1<=ADAS_H){indices[front_count]=i;front[front_count++]=*v;continue;}
            if(v->y0<rear_y||v->y1>front_y||v->score<.34f||v->cls<0||v->cls>4||
               !isfinite(v->x0)||!isfinite(v->x1)||!isfinite(v->y0)||!isfinite(v->y1)||
               v->x1<=v->x0||v->y1<=v->y0)continue;
            float center=(v->x0+v->x1)*.5f;
            rear_hud[0]|=v->x0<QUAD_LEFT_SEAM&&v->score>=(display_contiguous&&s->disp_occupied[0]?.34f:.45f);
            rear_hud[1]|=v->x1>QUAD_RIGHT_SEAM&&v->score>=(display_contiguous&&s->disp_occupied[1]?.34f:.45f);
            if(v->score<.45f)continue;
            rear_left|=v->x0<QUAD_LEFT_SEAM;
            rear_right|=v->x1>QUAD_RIGHT_SEAM;
            rear_near|=center>=QUAD_LEFT_SEAM&&center<=QUAD_RIGHT_SEAM&&
                (v->x1-v->x0)*(v->y1-v->y0)>640.f*(front_y-rear_y)*.12f;
        }
        d=front;n=front_count;
    }
    double dt=arrived-s->last_arrived;
    uint32_t delta=f->source_id-s->last_source;
    if(!s->have_frame||dt<=0||dt>.5||!delta||delta>15) memset(s,0,sizeof(*s));
    memset(out,0,sizeof(*out));
    memcpy(out->rear_hud_occupied,rear_hud,sizeof(rear_hud));
    out->roi_hits[0]=out->roi_hits[1]=-1;
    out->source_id=f->source_id; out->lead_index=-1; out->phase_time=now;
    out->age_ms=fmax(0,now-arrived)*1000;
    out->simulated=c->enabled==ADAS_CONTROL_SIMULATED; out->turn=c->turn; out->route=c->route;
    out->moving=out->simulated?c->moving:0; out->motion_known=out->simulated&&c->motion_known;
    out->road_profile=c->road_profile;
    out->ego_lane=c->ego_lane;
    out->auto_signal_fsm=out->simulated&&c->auto_signal_fsm;
    if(signal) out->signal=*signal;
    /* SEARCH promotion is explicit interactive opt-in; default remains ROI only. */
    int search_promoted=out->auto_signal_fsm&&signal&&signal->source==SIGNAL_SEARCH&&
        signal->stable&&isfinite(signal->confidence)&&signal->confidence>=.72f;
    out->light=signal&&signal->stable&&(signal->source==SIGNAL_ROI||search_promoted)?signal->candidate:LIGHT_UNKNOWN;
    /* HUD light: show any stable detector result (SEARCH or ROI) for competition display. */
    out->hud_light=out->light;
    if(signal&&signal->stable&&signal->candidate!=LIGHT_UNKNOWN) out->hud_light=signal->candidate;
    if(out->simulated&&c->light!=LIGHT_UNKNOWN) {
        out->light=c->light; out->hud_light=c->light;
        out->signal=(struct adas_signal){.candidate=c->light,.source=SIGNAL_MANUAL,.stable=1};
    }
    out->speed=out->simulated?c->speed:0; out->speed_known=out->simulated&&c->speed_known; out->speed_limit=c->speed_limit;
    out->lane_geometry=geometry(lanes,quad);
    if(out->lane_geometry) { out->corridor_left=at(&lanes[0],quad?QUAD_ROAD_MAX:1079); out->corridor_right=at(&lanes[1],quad?QUAD_ROAD_MAX:1079); }
    if(out->lane_geometry)out->lane_center_norm=(ADAS_W*.5f-(out->corridor_left+out->corridor_right)*.5f)/
        (out->corridor_right-out->corridor_left);
    uint32_t events=0; enum adas_level raw=ADAS_MONITOR; int immediate=0;
    if(!out->lane_geometry) events|=EVENT_NO_LANE;
    if(out->age_ms>250) { events|=EVENT_STALE; raw=ADAS_CAUTION; }
    if(f->layout==0x20100&&f->road[0]) {
        out->fpga_white_left=f->road[4]; out->fpga_white_right=f->road[5];
        out->fpga_edge_pixels=f->road[8]; out->fpga_morph_pixels=f->road[9];
        out->fpga_zebra_candidate=f->candidates[2].valid!=0;
        out->gray_mean=(float)f->road[1]/f->road[0]; out->dark_ratio=(float)f->road[2]/f->road[0];
        if(out->dark_ratio>.8f||out->gray_mean<40) { events|=EVENT_DARK; raw=ADAS_CAUTION; }
    }
    if(rear_near&&!(events&(EVENT_STALE|EVENT_DARK))) {events|=EVENT_REAR_NEAR;if(raw<ADAS_CAUTION)raw=ADAS_CAUTION;}
    int rear[2]={rear_left!=0,rear_right!=0};
    int occupied[2]={rear[0],rear[1]}; float lead_height=0;
    const char *adjacent_policy=getenv("ADAS_ADJACENT_POLICY");
    int cruise_adjacent=adjacent_policy&&!strcmp(adjacent_policy,"cruise");
    /* Front-camera trajectory candidate: adjacent-to-corridor crossing requires
     * same-class IoU correspondence, fresh geometry and stable boundaries. */
    if(out->lane_geometry&&s->previous_geometry&&dt>=.03&&dt<=.35&&!(events&EVENT_STALE)) {
        float boundary_delta=fabsf(at(&lanes[0],1000)-at(&s->previous_lanes[0],1000))+
            fabsf(at(&lanes[1],1000)-at(&s->previous_lanes[1],1000));
        if(boundary_delta>=64)s->cut_in_until=0;
        else for(int i=0;i<n;i++) {
            if(d[i].cls<0||d[i].cls>2||d[i].score<.5f||d[i].y1<front_height*.7f)continue;
            int match=-1;float overlap=.3f;
            for(int j=0;j<s->previous_count;j++)if(s->previous_objects[j].cls==d[i].cls&&s->previous_objects[j].score>=.5f) {
                float v=iou(&d[i],&s->previous_objects[j]);if(v>overlap){overlap=v;match=j;}
            }
            if(match<0)continue;
            const struct detection *previous=&s->previous_objects[match];
            float before=(previous->x0+previous->x1)*.5f,after=(d[i].x0+d[i].x1)*.5f;
            float py=fmaxf(front_top,previous->y1),cy=fmaxf(front_top,d[i].y1);
            float pl=at(&s->previous_lanes[0],py),pr=at(&s->previous_lanes[1],py);
            float cl=at(&lanes[0],cy),cr=at(&lanes[1],cy);
            float a=(before-pl)/fmaxf(pr-pl,1),b=(after-cl)/fmaxf(cr-cl,1);
            if((a<-.02f&&b>.04f&&b<.8f)||(a>1.02f&&b<.96f&&b>.2f)) {
                s->cut_in_side=a<0?-1:1;s->cut_in_until=now+.4;
            }
        }
        if(now<s->cut_in_until){events|=EVENT_CUT_IN;out->cut_in_side=s->cut_in_side;if(raw<ADAS_CAUTION)raw=ADAS_CAUTION;}
    } else s->cut_in_until=0;
    for(int i=0;i<n;i++) {
        float h=d[i].y1-d[i].y0;
        if(d[i].score<.35f||h<=0||d[i].x1<=d[i].x0||d[i].y1<front_height*.55f) continue;
        if(d[i].cls<3)out->vehicle_count++;else out->vru_count++;
        float y=fmaxf(front_top,d[i].y1);
        float left=out->lane_geometry?at(&lanes[0],y):ADAS_W*.35f;
        float right=out->lane_geometry?at(&lanes[1],y):ADAS_W*.65f;
        float width=right-left,center=(d[i].x0+d[i].x1)*.5f;
        if(out->lane_geometry&&d[i].y1>front_height*(cruise_adjacent?.72f:.65f)&&
           (!cruise_adjacent||(d[i].cls>=0&&d[i].cls<5&&d[i].score>=.50f&&h>=front_height*.06f))) {
            float required=cruise_adjacent?.35f:.25f;
            occupied[0]|=center<left&&fraction(left-width,left,d[i].x0,d[i].x1)>required;
            occupied[1]|=center>right&&fraction(right,right+width,d[i].x0,d[i].x1)>required;
        }
        if(center<left||center>right||fraction(left,right,d[i].x0,d[i].x1)<.25f) continue;
        if(h>lead_height) { lead_height=h; out->lead_index=i; }
        if(d[i].cls>=3) {
            events|=EVENT_PEDESTRIAN;
            if(raw<ADAS_CAUTION) raw=ADAS_CAUTION;
            if(d[i].y1>front_height*.88f&&h>front_height*.20f) raw=ADAS_DANGER;
        } else if(h>front_height*.25f&&d[i].y1>front_height*.78f) {
            events|=EVENT_NEAR;
            if(raw<ADAS_CAUTION) raw=ADAS_CAUTION;
            if(h>front_height*.42f&&d[i].y1>front_height*.93f) raw=ADAS_DANGER;
        }
    }
    if(out->lead_index>=0) {
        const struct detection *lead=&d[out->lead_index];
        if(s->have_lead&&dt>=.03&&dt<=.35&&lead->cls==s->lead.cls&&iou(lead,&s->lead)>.35f) {
            float old=s->lead.y1-s->lead.y0;
            float growth=(lead_height-old)/fmaxf(old,1)/(float)dt;
            out->approach_rate=growth;
            out->approach_known=1;
            if(growth>.5f&&lead_height>front_height*.20f&&lead->y1>front_height*.8f) { events|=EVENT_APPROACH; raw=ADAS_DANGER; }
        }
        s->lead=*lead; s->have_lead=1;
    } else s->have_lead=0;
    const struct candidate *z=&f->candidates[2];
    if(z->valid&&z->x1-z->x0<ADAS_W*3/4&&z->y1-z->y0<ADAS_H/2) events|=EVENT_ZEBRA;
    for(int side=0;side<2;side++) {
        /* Missing observations may draw a ghost, never restore validity. */
        if(events&(EVENT_STALE|EVENT_DARK)) {
            memset(&s->coast_lanes[side],0,sizeof(s->coast_lanes[side]));
            s->coast_miss[side]=0;
        } else if(lanes[side].valid&&lanes[side].type!=LANE_UNKNOWN&&
                  isfinite(lanes[side].slope)&&isfinite(lanes[side].intercept)) {
            struct lane current=lanes[side];
            if((f->source_kind&ADAS_SOURCE_STABLE_HUD)&&s->coast_lanes[side].valid&&
               fabsf(current.slope-s->coast_lanes[side].slope)<.35f) {
                current.slope=.75f*s->coast_lanes[side].slope+.25f*current.slope;
                current.intercept=.75f*s->coast_lanes[side].intercept+.25f*current.intercept;
            }
            s->coast_lanes[side]=current; s->coast_miss[side]=0;
        } else if(s->coast_lanes[side].valid&&s->coast_miss[side]<((f->source_kind&ADAS_SOURCE_STABLE_HUD)?22u:8u)) {
            s->coast_miss[side]++;
        } else memset(&s->coast_lanes[side],0,sizeof(s->coast_lanes[side]));
        out->coasting_valid[side]=s->coast_lanes[side].valid!=0;
        out->coasting_slope[side]=s->coast_lanes[side].slope;
        out->coasting_intercept[side]=s->coast_lanes[side].intercept;
        out->coasting_type[side]=s->coast_lanes[side].type;
        /* Use type even when valid==0 so presentational hold labels reach HUD. */
        enum lane_type type=lanes[side].type;
        out->display_class[side]=type==LANE_SOLID?LANE_CLASS_SOLID:type==LANE_DASHED?LANE_CLASS_DASHED:LANE_CLASS_UNKNOWN;
        out->display_frame[side]=f->source_id;
        out->display_held[side]=!lanes[side].valid&&type!=LANE_UNKNOWN;
        out->occupied[side]=occupied[side];
        out->hints[side]=type==LANE_SOLID?HINT_SOLID:type==LANE_DASHED?HINT_DASHED:HINT_UNKNOWN;
        /* Do not wipe held DASHED solely because geometry failed; EVENT_NO_LANE covers safety. */
        if((occupied[side]&&type==LANE_DASHED)||rear[side]) out->hints[side]=HINT_OCCUPIED;
        if(events&(EVENT_STALE|EVENT_DARK)) {
            out->hints[side]=HINT_UNKNOWN;
            out->display_class[side]=LANE_CLASS_UNKNOWN; out->display_held[side]=0;
        }
        int direction=side?1:-1;
        if(c->enabled&&c->turn==direction&&!(events&EVENT_STALE)&&
           (rear[side]||(out->lane_geometry&&lanes[side].valid&&
           (out->hints[side]==HINT_SOLID||out->hints[side]==HINT_OCCUPIED)))) {
            events|=EVENT_TURN_BLOCKED; if(raw<ADAS_CAUTION) raw=ADAS_CAUTION;
        }
        if(out->simulated&&out->lane_geometry&&lanes[side].valid&&c->forced_turn==direction&&
           now<c->forced_until&&type==LANE_SOLID&&!(events&(EVENT_STALE|EVENT_DARK))) {
            events|=EVENT_SOLID_FORCE; raw=ADAS_DANGER; immediate=1;
        }
    }
    if(out->lane_geometry) {
        float width=out->corridor_right-out->corridor_left;
        if(ADAS_W*.5f-out->corridor_left<width*.18f) events|=EVENT_EDGE_LEFT;
        if(out->corridor_right-ADAS_W*.5f<width*.18f) events|=EVENT_EDGE_RIGHT;
        if(events&(EVENT_EDGE_LEFT|EVENT_EDGE_RIGHT)) if(raw<ADAS_CAUTION) raw=ADAS_CAUTION;
    }
    if(out->light==LIGHT_RED&&!(events&EVENT_STALE)) {
        events|=EVENT_RED_STOP;
        if(out->simulated&&c->motion_known&&c->moving) { events|=EVENT_RED_MOVING; raw=ADAS_DANGER; immediate=1; }
    }
    if(out->simulated) {
        if(c->speed_known&&c->speed>c->speed_limit) { events|=EVENT_OVERSPEED; if(raw<ADAS_CAUTION) raw=ADAS_CAUTION; }
    }
    if((events&EVENT_STALE)&&raw<ADAS_CAUTION) raw=ADAS_CAUTION;
    if(raw!=s->pending) { s->pending=raw; s->hits=1; s->pending_since=now; }
    else if(s->hits<1000) s->hits++;
    int confirmed=immediate||(s->hits>=2&&now-s->pending_since>=.08);
    if(raw>s->level&&confirmed) s->level=raw;
    if(raw==s->level&&raw!=ADAS_MONITOR) {
        s->held_events=events; s->hold_until=now+(raw==ADAS_DANGER?.8:.4);
    } else if(raw<s->level&&now>=s->hold_until) { s->level=raw; s->held_events=events; }
    out->level=s->level;
    /* Keep the danger level briefly, but FORCE always requires current valid geometry. */
    out->events=events|(s->level>raw?(s->held_events&~EVENT_SOLID_FORCE):0);
    out->road_ready=1;
    out->road_scene=!out->lane_geometry?ROAD_UNCERTAIN:ROAD_LANE_MONITOR;
    if(out->vehicle_count>=3)out->road_scene=ROAD_DENSE_TRAFFIC;
    if(events&EVENT_CUT_IN)out->road_scene=ROAD_CUT_IN_WATCH;
    if((events&EVENT_ZEBRA)||(signal&&signal->stable))out->road_scene=ROAD_JUNCTION_CANDIDATE;
    if(events&EVENT_PEDESTRIAN)out->road_scene=ROAD_VRU_WATCH;
    if(events&EVENT_DARK)out->road_scene=ROAD_LOW_VISIBILITY;
    if(events&EVENT_STALE)out->road_scene=ROAD_UNCERTAIN;
    out->road_action=ROAD_OBSERVE;
    if(out->events&EVENT_STALE)out->road_action=ROAD_CHECK_INPUT;
    else if(out->events&EVENT_PEDESTRIAN)out->road_action=ROAD_SLOW_FOR_VRU;
    else if(out->light==LIGHT_RED||(out->events&(EVENT_RED_STOP|EVENT_RED_MOVING)))out->road_action=c->motion_known&&!c->moving?ROAD_KEEP_STOPPED:ROAD_PREPARE_STOP;
    else if(out->events&(EVENT_NEAR|EVENT_APPROACH|EVENT_CUT_IN))out->road_action=ROAD_INCREASE_GAP;
    else if(out->events&(EVENT_SOLID_FORCE|EVENT_TURN_BLOCKED))out->road_action=ROAD_NO_LANE_CHANGE;
    else if(out->events&EVENT_DARK)out->road_action=ROAD_SLOW_VISIBILITY;
    else if(out->lane_geometry&&(out->events&(EVENT_EDGE_LEFT|EVENT_EDGE_RIGHT)))out->road_action=ROAD_KEEP_CENTER;
    else if(out->hud_light==LIGHT_RED&&out->light!=LIGHT_RED)out->road_action=ROAD_WATCH_SIGNAL;
    else if(out->hud_light==LIGHT_YELLOW)out->road_action=ROAD_YELLOW_WATCH;
    else if(events&EVENT_ZEBRA)out->road_action=ROAD_WATCH_CROSSING;
    else if(!out->lane_geometry)out->road_action=ROAD_CHECK_LANE;
    else if(out->vehicle_count>=3)out->road_action=ROAD_INCREASE_GAP;
    s->previous_count=n>MAX_DETECTIONS?MAX_DETECTIONS:n<0?0:n;
    if(s->previous_count)memcpy(s->previous_objects,d,(size_t)s->previous_count*sizeof(*d));
    memcpy(s->previous_lanes,lanes,sizeof(s->previous_lanes));s->previous_geometry=out->lane_geometry;
    if(quad&&out->lead_index>=0)out->lead_index=indices[out->lead_index];
    s->last_source=f->source_id; s->last_arrived=arrived; s->have_frame=1;
}
void adas_state_update(struct adas_state *s,const struct features *f,const struct detection *d,int n,
    const struct lane lanes[2],const struct adas_controls *c,const struct adas_signal *signal,
    double arrived,double now,struct adas_status *out) {
    adas_state_update_with_rear(s,f,d,n,lanes,c,signal,arrived,now,0,0,out);
}

static uint32_t hud_primary(uint32_t events) {
    static const uint32_t priority[]={EVENT_RED_MOVING,EVENT_SOLID_FORCE,EVENT_PEDESTRIAN,
        EVENT_APPROACH,EVENT_NEAR,EVENT_STALE,EVENT_DARK,EVENT_CUT_IN,EVENT_REAR_NEAR,
        EVENT_TURN_BLOCKED,EVENT_RED_STOP,EVENT_NO_LANE,EVENT_EDGE_LEFT,EVENT_EDGE_RIGHT,
        EVENT_OVERSPEED,EVENT_ZEBRA};
    for(unsigned i=0;i<sizeof(priority)/sizeof(priority[0]);i++)if(events&priority[i])return priority[i];
    return 0;
}
void adas_hud_prepare(struct adas_state *s,const struct features *f,const struct lane lanes[2],
    const struct adas_controls *c,int object_count,struct adas_status *out) {
    struct hud_view *v=&out->hud;memset(v,0,sizeof(*v));
    if(!(f->source_kind&ADAS_SOURCE_STABLE_HUD))return;
    v->enabled=1;
    int quad=(f->source_kind&ADAS_SOURCE_QUAD)!=0;
    int unhealthy=(out->events&(EVENT_STALE|EVENT_DARK))!=0;
    for(int side=0;side<2;side++) {
        int usable=!unhealthy&&(lanes[side].valid||out->coasting_valid[side]);
        enum lane_type t=lanes[side].valid&&lanes[side].type!=LANE_UNKNOWN?lanes[side].type:
            out->coasting_valid[side]?out->coasting_type[side]:LANE_UNKNOWN;
        if(s->hud_type[side]==LANE_SOLID&&t==LANE_DASHED&&
           (!lanes[side].valid||lanes[side].type!=LANE_DASHED||lanes[side].coverage<.32f))t=LANE_SOLID;
        if(!usable||t==LANE_UNKNOWN) {s->hud_type[side]=s->type_pending[side]=LANE_UNKNOWN;s->type_hits[side]=0;}
        else if(s->hud_type[side]==LANE_UNKNOWN) s->hud_type[side]=t;
        else if(s->hud_type[side]!=t) {
            if(s->type_pending[side]!=t){s->type_pending[side]=t;s->type_hits[side]=1;}
            else if(s->type_hits[side]<6)s->type_hits[side]++;
            if(s->type_hits[side]>=6)s->hud_type[side]=t;
        } else {s->type_hits[side]=0;s->type_pending[side]=t;}
        v->side_valid[side]=usable;v->type[side]=usable?s->hud_type[side]:LANE_UNKNOWN;
    }
    v->ready=!unhealthy&&(out->lane_geometry||(v->side_valid[0]&&v->side_valid[1]));
    for(int side=0;side<2;side++) {
        int raw=out->occupied[side]||(quad&&out->rear_hud_occupied[side]);
        if(unhealthy||object_count==0||(!quad&&!v->ready)){s->occ_hold[side]=0;s->disp_occupied[side]=0;}
        else if(raw){s->occ_hold[side]=8;s->disp_occupied[side]=1;}
        else if(s->occ_hold[side]){s->occ_hold[side]--;s->disp_occupied[side]=1;}
        else s->disp_occupied[side]=0;
        /* Rear camera observations remain visible even before lane geometry confirms. */
        if(!unhealthy&&raw){s->disp_occupied[side]=1;s->occ_hold[side]=8;}
        v->occupied[side]=s->disp_occupied[side];
        v->hints[side]=v->occupied[side]?HINT_OCCUPIED:v->type[side]==LANE_SOLID?HINT_SOLID:
            v->type[side]==LANE_DASHED?HINT_DASHED:HINT_UNKNOWN;
    }
    uint32_t events=out->events;
    if(v->ready)events&=~EVENT_NO_LANE;
    events&=~EVENT_TURN_BLOCKED;
    int turn_side=c->turn>0?1:0;
    v->turn_blocked=quad&&!unhealthy&&c->turn&&v->occupied[turn_side];
    if(c->turn&&(!v->ready||v->hints[turn_side]!=HINT_DASHED))events|=EVENT_TURN_BLOCKED;
    if(c->enabled==ADAS_CONTROL_SIMULATED&&c->turn&&c->forced_turn==c->turn&&out->phase_time<c->forced_until&&
       !unhealthy&&v->side_valid[turn_side]&&v->type[turn_side]==LANE_SOLID) {
        events|=EVENT_SOLID_FORCE;v->force_from_hold=!(out->events&EVENT_SOLID_FORCE);
    }
    uint32_t candidate=hud_primary(events);
    if(quad&&(events&EVENT_APPROACH)&&out->approach_rate>.8f) {if(s->approach_hits<2)s->approach_hits++;}
    else s->approach_hits=0;
    int critical=(candidate&(EVENT_RED_MOVING|EVENT_SOLID_FORCE|EVENT_STALE|EVENT_DARK))!=0||
        (candidate==EVENT_APPROACH&&(!quad||s->approach_hits>=2))||
        ((candidate&(EVENT_NEAR|EVENT_PEDESTRIAN))&&out->level==ADAS_DANGER);
    int key_change=s->hud_have&&(c->turn!=s->hud_turn||c->ego_lane!=s->hud_ego||c->moving!=s->hud_moving);
    if(s->hud_pending!=candidate){s->hud_pending=candidate;s->hud_hits=1;}
    else if(s->hud_hits<1000)s->hud_hits++;
    int force_end=s->hud_event==EVENT_SOLID_FORCE&&candidate!=EVENT_SOLID_FORCE;
    if(!s->hud_have||critical||key_change||force_end||(s->hud_age>=6&&s->hud_hits>=3)) {
        if(s->hud_event!=candidate)s->hud_age=0;
        s->hud_event=candidate;
    }
    s->hud_age++;s->hud_have=1;s->hud_turn=c->turn;s->hud_ego=c->ego_lane;s->hud_moving=c->moving;
    v->events=s->hud_event;
    v->level=v->events&(EVENT_SOLID_FORCE|EVENT_RED_MOVING|EVENT_APPROACH)?ADAS_DANGER:
        v->events&~(EVENT_NO_LANE|EVENT_RED_STOP)?ADAS_CAUTION:ADAS_MONITOR;
    if((v->events&out->events)&&out->level>v->level)v->level=out->level;
}
