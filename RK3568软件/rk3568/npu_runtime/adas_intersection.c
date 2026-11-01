#include "adas_intersection.h"
#include "lane_cnn_v2.h"
#include <math.h>
#include <string.h>

int intersection_key_snapshot(const volatile uint32_t *map,uint32_t *sequence,
    uint32_t *left,uint32_t *right,unsigned *down) {
    if(!map)return -1;
    const volatile uint32_t *p=map+LANE_KEY_BAR_BASE/4;
    for(int i=0;i<50;i++) {
        uint32_t seq=p[3];if(seq&1)continue;
        __sync_synchronize();uint32_t magic=p[0],l=p[1],r=p[2];
        __sync_synchronize();if(seq!=p[3])continue;
        if((magic&UINT32_C(0x3fffffff))!=UINT32_C(0x3159454b))return -1;
        *sequence=seq;*left=l;*right=r;*down=magic>>30;return 1;
    }
    return 0;
}
int intersection_key_step(struct intersection_keys *s,uint32_t seq,uint32_t left,
    uint32_t right,unsigned down,double t) {
    if(!isfinite(t)||down>3)return JUNCTION_NONE;
    int had=s->initialized;
    int reset=!s->initialized||t<=s->last||t-s->last>.5||
        (uint32_t)(seq-s->sequence)>=UINT32_C(0x80000000)||
        (uint32_t)(left-s->left)>=UINT32_C(0x80000000)||
        (uint32_t)(right-s->right)>=UINT32_C(0x80000000)||
        (seq==s->sequence&&(left!=s->left||right!=s->right));
    unsigned edges=(left!=s->left?1u:0u)|(right!=s->right?2u:0u);
    if(reset) {
        memset(s,0,sizeof(*s));s->initialized=1;s->ignore=down!=0;
    }
    s->left=left;s->right=right;s->sequence=seq;s->last=t;
    unsigned previous=s->down;s->down=down;
    if(reset)return had?-1:JUNCTION_NONE;
    if(s->ignore){if(!down)s->ignore=0;return JUNCTION_NONE;}
    if(!s->pending&&(down||edges)) {
        s->pending=down|edges;s->pressed=t;s->stopped=0;
        s->chord=down==3||edges==3;
    }
    if(!s->pending)return JUNCTION_NONE;
    /* Singles wait for release: the second physical key cannot cause a turn
     * before a chord has been classified. Historical counts never fire at boot. */
    if(down==3) {
        if(!s->chord){s->chord=1;s->pressed=t;}
        if(!s->stopped&&t-s->pressed>=1.){s->stopped=1;return JUNCTION_STOP;}
        return JUNCTION_NONE;
    }
    if(s->chord) {
        if(previous==3||(!down&&edges==3)) {
            int result=s->stopped?JUNCTION_NONE:JUNCTION_STRAIGHT;
            s->pending=0;s->chord=0;s->ignore=down!=0;return result;
        }
        return JUNCTION_NONE;
    }
    if(!down) {
        int result=s->pending==1?JUNCTION_LEFT:s->pending==2?JUNCTION_RIGHT:JUNCTION_NONE;
        s->pending=0;return result;
    }
    return JUNCTION_NONE;
}
static float overlap(const struct detection *a,const struct detection *b) {
    float area=fmaxf(0,fminf(a->x1,b->x1)-fmaxf(a->x0,b->x0))*
        fmaxf(0,fminf(a->y1,b->y1)-fmaxf(a->y0,b->y0));
    return area/fmaxf(1,(a->x1-a->x0)*(a->y1-a->y0)+(b->x1-b->x0)*(b->y1-b->y0)-area);
}
static int road_user(const struct detection *d) {
    return d->cls>=3&&d->cls<=5&&isfinite(d->score)&&d->score>=.35f&&
        isfinite(d->x0)&&isfinite(d->x1)&&isfinite(d->y0)&&isfinite(d->y1)&&
        d->x1>d->x0&&d->y1>d->y0;
}
static int conflict(const struct detection *d) {
    if(!road_user(d)||d->x0<0||d->x1>ADAS_W||d->y0<0||d->y1>ADAS_H||
       d->y1<ADAS_H*.48f||d->y1-d->y0<ADAS_H*.04f)return 0;
    /* Image-space crossing region; installation/video calibration is required. */
    float depth=(d->y1/ADAS_H-.48f)/.52f;
    float left=ADAS_W*(.30f-.15f*depth),right=ADAS_W-left;
    float foot=(d->x0+d->x1)*.5f;
    return foot>=left&&foot<=right;
}
void intersection_update(struct intersection_state *s,int command,const struct detection *d,
    int n,const struct lane lanes[2],const struct adas_status *status,double t,struct intersection_view *v) {
    memset(v,0,sizeof(*v));v->enabled=1;
    for(int i=0;i<3;i++)v->permission[i]=JUNCTION_UNCONFIRMED;
    if(!isfinite(t))return;
    if(!s->initialized||t<=s->last||t-s->last>.5) {
        memset(s,0,sizeof(*s));key_drive_reset(&s->drive);s->origin=t;s->initialized=1;
    }
    s->last=t;
    v->stale=!isfinite(status->age_ms)||status->age_ms>250||(status->events&EVENT_STALE);
    if(v->stale){key_drive_reset(&s->drive);s->subject_count=0;return;}
    if(command<JUNCTION_NONE||command>JUNCTION_RIGHT)command=JUNCTION_NONE;
    int confirmed=status->signal.source==SIGNAL_ROI&&status->signal.stable&&
        isfinite(status->signal.confidence)&&status->signal.confidence>=.72f&&
        status->light!=LIGHT_UNKNOWN&&status->signal.candidate==status->light;
    /* The button is an intention/motion demonstration, never measured speed. */
    key_drive_step(&s->drive,(int)((t-s->origin)*1000),status->light,confirmed,0,command);
    v->command=s->drive.manual?(s->drive.motion==1?JUNCTION_STOP:s->drive.route+3):JUNCTION_NONE;
    v->motion=s->drive.motion;v->red_alarm=s->drive.alarm;
    if(n<0||!d)n=0;
    if(n>MAX_DETECTIONS)n=MAX_DETECTIONS;
    int visible=0;
    for(int j=0;j<s->subject_count;j++) {
        int best=-1;float score=.20f;
        for(int i=0;i<n;i++)if(road_user(&d[i])&&d[i].cls==s->subjects[j].cls) {
            float q=overlap(&d[i],&s->subjects[j]);if(q>score){score=q;best=i;}
        }
        if(best>=0){s->subjects[j]=d[best];visible|=conflict(&d[best]);}
    }
    if(visible)s->last_visible=t;
    for(int i=0;i<n;i++)if(conflict(&d[i])) {
        int found=0;
        for(int j=0;j<s->subject_count;j++)if(overlap(&d[i],&s->subjects[j])>.20f)found=1;
        if(!found&&s->subject_count<16)s->subjects[s->subject_count++]=d[i];
        s->last_visible=t;
    }
    if(s->subject_count&&t-s->last_visible>=.25)s->subject_count=0;
    v->pedestrian_present=s->subject_count>0;
    v->pedestrian_alarm=v->pedestrian_present&&s->drive.motion!=1;
    uint32_t unsafe=EVENT_STALE|EVENT_NEAR|EVENT_APPROACH|EVENT_TURN_BLOCKED|
        EVENT_SOLID_FORCE|EVENT_CUT_IN|EVENT_REAR_NEAR;
    for(int i=0;i<3;i++) {
        v->permission[i]=JUNCTION_UNCONFIRMED;
        if(v->red_alarm||v->pedestrian_present||(confirmed&&status->light==LIGHT_RED))
            v->permission[i]=JUNCTION_BLOCKED;
    }
    for(int side=0;side<2;side++)if(status->lane_geometry&&lanes[side].valid&&
        (lanes[side].type==LANE_SOLID||status->occupied[side]))
        v->permission[side?2:0]=JUNCTION_BLOCKED;
    /* No live lane-arrow observation exists in this baseline. Directional
     * permissions stay unconfirmed; a bound green can advise straight only. */
    if(confirmed&&status->light==LIGHT_GREEN&&!v->pedestrian_present&&
       !(status->events&unsafe))v->permission[1]=JUNCTION_ALLOWED;
}
void intersection_present(const struct intersection_view *v,struct adas_status *s) {
    if(v->stale)return;
    if(v->red_alarm)s->events|=EVENT_RED_MOVING;
    if(v->pedestrian_alarm)s->events|=EVENT_PEDESTRIAN;
    if(v->red_alarm||v->pedestrian_alarm) {
        s->level=ADAS_DANGER;
        if(s->hud.enabled){s->hud.events|=s->events;s->hud.level=ADAS_DANGER;}
    }
}
