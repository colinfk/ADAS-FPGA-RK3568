#include "adas_intersection.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
static void step(struct intersection_state *j,struct intersection_view *v,
                 struct detection *d,struct lane lanes[2],struct adas_status *s,double t) {
    intersection_update(j,JUNCTION_NONE,d,d?1:0,lanes,s,t,v);
}
int main(void) {
    struct intersection_state j={0};struct intersection_view v;
    struct lane lanes[2]={{0}};struct adas_status s={0};
    struct detection d={.x0=920,.x1=1000,.y0=420,.y1=560,.score=.8f,.cls=5};
    step(&j,&v,&d,lanes,&s,1);assert(v.pedestrian_alarm);
    /* The same road user and timestamp path must survive unreliable lane fits. */
    s.lane_geometry=1;lanes[0].slope=-.5f;lanes[0].intercept=1700;
    lanes[1].slope=.5f;lanes[1].intercept=1800;
    step(&j,&v,&d,lanes,&s,1.1);assert(v.pedestrian_alarm);
    /* A still-visible matched person exiting the region clears after 250 ms. */
    memset(&j,0,sizeof(j));s.lane_geometry=0;
    d=(struct detection){.x0=1260,.x1=1420,.y0=400,.y1=560,.score=.8f,.cls=5};
    step(&j,&v,&d,lanes,&s,2);assert(v.pedestrian_alarm);
    d.x0=1300;d.x1=1460;
    step(&j,&v,&d,lanes,&s,2.1);assert(v.pedestrian_alarm);
    step(&j,&v,&d,lanes,&s,2.3);assert(!v.pedestrian_alarm);
    /* Roadside and tiny far boxes stay visible to YOLO without causing conflict. */
    d=(struct detection){.x0=1700,.x1=1800,.y0=500,.y1=800,.score=.8f,.cls=5};
    step(&j,&v,&d,lanes,&s,2.4);assert(!v.pedestrian_alarm);
    d.x0=940;d.x1=960;d.y0=520;d.y1=550;
    step(&j,&v,&d,lanes,&s,2.5);assert(!v.pedestrian_alarm);
    d.y0=400;d.y1=560;d.x0=NAN;
    step(&j,&v,&d,lanes,&s,2.6);assert(!v.pedestrian_alarm);
    d.x0=920;d.x1=1000;
    step(&j,&v,&d,lanes,&s,2.7);assert(v.pedestrian_alarm);
    intersection_update(&j,JUNCTION_STOP,&d,1,lanes,&s,2.8,&v);assert(!v.pedestrian_alarm&&v.pedestrian_present);
    assert(v.permission[1]==JUNCTION_BLOCKED);
    intersection_update(&j,JUNCTION_STRAIGHT,&d,1,lanes,&s,2.85,&v);assert(v.pedestrian_alarm);
    s.age_ms=300;step(&j,&v,&d,lanes,&s,2.9);assert(v.stale&&!v.pedestrian_alarm);
    puts("INTERSECTION_ROI_PASS");
}
