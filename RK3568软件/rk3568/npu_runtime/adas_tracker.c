#include "adas_tracker.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#define CUT_SAMPLES (16*9)
static void signature(const uint8_t *image,uint8_t out[CUT_SAMPLES]) {
    for(int y=0;y<9;y++) for(int x=0;x<16;x++) {
        size_t offset=((size_t)(y*120+60)*ADAS_W+x*120+60)*2;
        unsigned p=image[offset]|(unsigned)image[offset+1]<<8;
        unsigned r=(p>>11)*8,g=((p>>5)&63)*4,b=(p&31)*8;
        out[y*16+x]=(uint8_t)((r+2*g+b)/4);
    }
}
struct adas_box_display {
    struct detection boxes[MAX_DETECTIONS];
    struct track_view views[MAX_DETECTIONS];uint64_t next_id;int reset;
    unsigned missed[MAX_DETECTIONS];
    int count,have_frame;
    uint32_t source;
    double arrived;
    uint8_t signature[CUT_SAMPLES];
};
struct adas_box_display *adas_box_display_create(void) {return calloc(1,sizeof(struct adas_box_display));}
void adas_box_display_destroy(struct adas_box_display *s) {free(s);}
static float display_iou(const struct detection *a,const struct detection *b) {
    float w=fmaxf(0,fminf(a->x1,b->x1)-fmaxf(a->x0,b->x0));
    float h=fmaxf(0,fminf(a->y1,b->y1)-fmaxf(a->y0,b->y0));
    return w*h/fmaxf(1,(a->x1-a->x0)*(a->y1-a->y0)+(b->x1-b->x0)*(b->y1-b->y0)-w*h);
}
int adas_box_display_update(struct adas_box_display *s,const uint8_t *image,uint32_t source,
    double arrived,int quad,const struct detection *raw,int count,struct detection shown[MAX_DETECTIONS],int indices[MAX_DETECTIONS]) {
    if(!s||!image||!shown||!indices||count<0||count>MAX_DETECTIONS||(count&&!raw)||!isfinite(arrived))return -1;
    for(int i=0;i<count;i++)if(!isfinite(raw[i].x0)||!isfinite(raw[i].x1)||!isfinite(raw[i].y0)||
        !isfinite(raw[i].y1)||!isfinite(raw[i].score)||raw[i].x0<0||raw[i].y0<0||raw[i].x1>ADAS_W||
        raw[i].y1>ADAS_H||raw[i].x1<raw[i].x0||raw[i].y1<raw[i].y0||raw[i].score<0||raw[i].score>1.01f||raw[i].cls<0||raw[i].cls>=6)return -1;
    uint8_t sig[CUT_SAMPLES];signature(image,sig);
    uint32_t gap=source-s->source;unsigned change=0;
    if(s->have_frame)for(int i=0;i<CUT_SAMPLES;i++)change+=(unsigned)abs((int)sig[i]-s->signature[i]);
    s->reset=!s->have_frame||!gap||gap>15||arrived<=s->arrived||arrived-s->arrived>.5||change>CUT_SAMPLES*55u;
    if(s->reset)s->count=0;
    unsigned char used[MAX_DETECTIONS]={0};unsigned missed[MAX_DETECTIONS]={0};int n=0;
    struct track_view views[MAX_DETECTIONS]={{0}};
    for(int i=0;i<count&&n<MAX_DETECTIONS;i++) {
        const struct detection *d=&raw[i];if(d->score<.35f||d->x1-d->x0<2||d->y1-d->y0<2)continue;
        int match=-1;float best=.45f;
        for(int j=0;j<s->count;j++)if(!used[j]&&d->cls==s->boxes[j].cls&&
            (!quad||((d->y0>=(quad==2?270:QUAD_FRONT_Y0))==(s->boxes[j].y0>=(quad==2?270:QUAD_FRONT_Y0))))) {
            float overlap=display_iou(d,&s->boxes[j]);if(overlap>=best){best=overlap;match=j;}
        }
        if(match<0&&d->score<.45f)continue;
        shown[n]=*d;indices[n]=i;
        if(match<0) {if(s->next_id==UINT64_MAX)return -1;views[n].id=++s->next_id;}
        else views[n]=s->views[match];
        if(views[n].hits<3)views[n].hits++;
        views[n].confirmed=views[n].hits>=3;
        if(match>=0) {
            const struct detection *p=&s->boxes[match];used[match]=1;
            shown[n].x0=.7f*p->x0+.3f*d->x0;shown[n].x1=.7f*p->x1+.3f*d->x1;
            shown[n].y0=.7f*p->y0+.3f*d->y0;shown[n].y1=.7f*p->y1+.3f*d->y1;
            shown[n].score=.7f*p->score+.3f*d->score;
            double dt=arrived-s->arrived;
            views[n].vx=(float)((shown[n].x0+shown[n].x1-p->x0-p->x1)/(2*fmax(.001,dt)));
            views[n].vy=(float)((shown[n].y0+shown[n].y1-p->y0-p->y1)/(2*fmax(.001,dt)));
        }
        n++;
    }
    for(int j=0;j<s->count&&n<MAX_DETECTIONS;j++)if(!used[j]&&s->missed[j]<2) {
        shown[n]=s->boxes[j];views[n]=s->views[j];indices[n]=-1;missed[n]=s->missed[j]+1;n++;
    }
    memcpy(s->views,views,sizeof(views));
    memcpy(s->boxes,shown,(size_t)n*sizeof(*shown));memcpy(s->missed,missed,sizeof(missed));
    memcpy(s->signature,sig,sizeof(sig));s->count=n;s->source=source;s->arrived=arrived;s->have_frame=1;
    return n;
}
struct adas_tracker {struct adas_box_display display;};
struct adas_tracker *adas_tracker_create(void) {return calloc(1,sizeof(struct adas_tracker));}
void adas_tracker_destroy(struct adas_tracker *s) {free(s);}
int adas_tracker_update(struct adas_tracker *s,const uint8_t *image,uint32_t source,double arrived,
    const struct detection *raw,int count,struct detection shown[MAX_DETECTIONS],struct track_view views[MAX_DETECTIONS]) {
    if(!s||!shown||!views||count<0||count>MAX_DETECTIONS||(count&&!raw))return -1;
    struct detection matched[MAX_DETECTIONS];int indices[MAX_DETECTIONS];
    int n=adas_box_display_update(&s->display,image,source,arrived,0,raw,count,matched,indices);
    if(n<0)return -1;
    if(count)memcpy(shown,raw,(size_t)count*sizeof(*shown));
    memset(views,0,(size_t)count*sizeof(*views));
    for(int j=0;j<n;j++)if(indices[j]>=0){int i=indices[j];shown[i]=matched[j];shown[i].score=raw[i].score;views[i]=s->display.views[j];}
    return s->display.reset;
}
