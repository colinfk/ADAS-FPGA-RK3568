#include "../adas_hud.h"
#include "../lane_cnn_v2.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static struct features feature(void) {
    struct features f={.layout=0x20100,.source_id=100};
    f.road[0]=ADAS_W*ADAS_H; f.road[1]=110*f.road[0];
    return f;
}
static void scene(struct adas_status *out,const struct lane lanes[2],
                  const struct detection *d,int n,const struct adas_controls *c,const struct adas_signal *lamp) {
    struct features f=feature(); struct adas_state state={0};
    adas_state_update(&state,&f,d,n,lanes,c,lamp,1,1,out);
    f.source_id++; adas_state_update(&state,&f,d,n,lanes,c,lamp,1.1,1.1,out);
}
static void render(const char *dir,const char *name,const uint8_t *base,uint8_t *image,
                   const struct lane lanes[2],const struct adas_status *s) {
    memcpy(image,base,IMAGE_BYTES);
    struct features f=feature();
    adas_hud(image,&f,NULL,0,lanes,s,0,"FPS 23.3 | NPU 33.0 MS | OBJECTS 2");
    int fresh=!s->display_held[0]&&!s->display_held[1]&&
        s->display_class[0]!=LANE_CLASS_UNKNOWN&&s->display_class[1]!=LANE_CLASS_UNKNOWN;
    adas_hud_cnn(image,s,fresh?"SYNC | F 100 | GAP 900/800 | FUSED 0":"WAIT SYNC",fresh);
    char path[1024]; snprintf(path,sizeof(path),"%s/%s.rgb565",dir,name);
    FILE *file=fopen(path,"wb"); assert(file);
    assert(fwrite(image,1,IMAGE_BYTES,file)==IMAGE_BYTES); assert(!fclose(file));
}
int main(int argc,char **argv) {
    assert(argc==2);
    struct lane_cnn_display display={0};
    struct lane_cnn_snapshot snap={.sequence=2,.frame_id=100,.valid={1,1},.cls={2,1}};
    lane_cnn_display_update(&display,&snap,0,0,1); assert(display.cls==LANE_CLASS_SOLID);
    snap.cls[0]=1;
    for(int i=0;i<5;i++) lane_cnn_display_update(&display,&snap,0,.01,0);
    assert(display.count==1&&display.cls==LANE_CLASS_SOLID); /* A replayed result cannot win votes. */
    snap.sequence=4; lane_cnn_display_update(&display,&snap,0,.02,0);
    snap.sequence=6; lane_cnn_display_update(&display,&snap,0,.03,0);
    assert(display.cls==LANE_CLASS_DASHED);
    for(int i=0;i<12;i++) {
        lane_cnn_display_update(&display,NULL,0,.04+i*.01,0);
        assert(display.cls==LANE_CLASS_DASHED&&display.held);
    }
    lane_cnn_display_update(&display,NULL,0,.16,0); assert(display.cls==LANE_CLASS_UNKNOWN&&!display.held);
    snap.cls[0]=0; snap.sequence=8;
    lane_cnn_display_update(&display,&snap,0,1,1); assert(display.cls==LANE_CLASS_NONE);
    lane_cnn_display_update(&display,NULL,0,1.3,0); assert(display.cls==LANE_CLASS_UNKNOWN);
    snap.valid[0]=0; lane_cnn_display_update(&display,&snap,0,1.31,0); assert(display.cls==LANE_CLASS_INVALID);
    snap.valid[0]=1; snap.cls[0]=3; lane_cnn_display_update(&display,&snap,0,1.32,0); assert(display.cls==LANE_CLASS_INVALID);

    struct lane_history history={0}; struct lane lane={-.8f,1320,1,0,1,LANE_SOLID};
    for(int i=0;i<3;i++) { lane.valid=1; lane.type=LANE_SOLID; adas_lane_stabilize(&history,&lane,i==0); }
    for(int i=0;i<13;i++) {
        lane=(struct lane){-.8f,i%2?1440:1200,1,0,1,LANE_DASHED};
        adas_lane_stabilize(&history,&lane,0);
        assert(!lane.valid);
    }
    assert(lane.type==LANE_UNKNOWN); /* Geometry changes cannot retain old SOLID forever. */
    assert(history.stable==LANE_UNKNOWN&&history.pending==LANE_UNKNOWN&&history.hits==0&&history.hold_miss==0);
    for(int i=0;i<3;i++) {
        lane=(struct lane){-.8f,1320,1,0,1,LANE_DASHED};
        adas_lane_stabilize(&history,&lane,0);
        assert(lane.valid==(unsigned)(i==2));
    }
    assert(lane.type==LANE_DASHED);
    for(int i=0;i<13;i++) {
        lane=(struct lane){0};
        adas_lane_stabilize(&history,&lane,0);
        assert(!lane.valid&&lane.slope==0&&lane.intercept==0);
        assert(lane.type==(i<12?LANE_DASHED:LANE_UNKNOWN));
    }
    assert(history.pending==LANE_UNKNOWN&&history.hits==0&&history.hold_miss==0);
    for(int i=0;i<3;i++) { lane=(struct lane){-.8f,1320,1,0,1,LANE_SOLID}; adas_lane_stabilize(&history,&lane,i==0); }
    lane.valid=1; lane.type=LANE_DASHED; adas_lane_stabilize(&history,&lane,0);
    lane.valid=0; lane.type=LANE_UNKNOWN; adas_lane_stabilize(&history,&lane,0); assert(history.hits==0);
    for(int i=0;i<2;i++) { lane.valid=1; lane.type=LANE_DASHED; adas_lane_stabilize(&history,&lane,0); assert(lane.type==LANE_SOLID&&!lane.valid); }
    lane.valid=1; lane.type=LANE_DASHED; adas_lane_stabilize(&history,&lane,0); assert(lane.type==LANE_DASHED&&lane.valid);

    struct lane lanes[2]={{-.8f,1320,1,0,1,LANE_DASHED},{.8f,600,1,0,1,LANE_SOLID}};
    struct adas_controls c; adas_controls_init(&c,0);
    struct adas_signal lamp={0}; struct adas_status status;
    const uint32_t active_events[]={EVENT_NO_LANE,EVENT_SOLID_FORCE,EVENT_RED_MOVING,EVENT_STALE,
        EVENT_TURN_BLOCKED,EVENT_OVERSPEED,EVENT_PEDESTRIAN,EVENT_NEAR,EVENT_APPROACH,
        EVENT_EDGE_LEFT,EVENT_EDGE_RIGHT,EVENT_DARK,EVENT_RED_STOP};
    for(unsigned i=0;i<sizeof(active_events)/sizeof(active_events[0]);i++)
        for(int level=ADAS_MONITOR;level<=ADAS_DANGER;level++) {
            status=(struct adas_status){.events=active_events[i],.level=(enum adas_level)level};
            assert(strcmp(adas_hud_event_name(&status),"OK"));
        }
    struct detection adjacent={150,600,480,1000,.9f,0};
    scene(&status,lanes,&adjacent,1,&c,&lamp);
    assert(status.occupied[0]&&status.display_class[0]==LANE_CLASS_DASHED);
    assert(status.hints[0]==HINT_OCCUPIED);
    uint8_t *base=malloc(IMAGE_BYTES),*image=malloc(IMAGE_BYTES),*second=malloc(IMAGE_BYTES); assert(base&&image&&second);
    for(int y=0;y<ADAS_H;y++)for(int x=0;x<ADAS_W;x++) {
        unsigned color=y<640?0x8410:0x4208;
        if(y>=720&&(abs(x-(1320-y*4/5))<4||abs(x-(600+y*4/5))<4)) color=0xffff;
        base[(y*ADAS_W+x)*2]=(uint8_t)color; base[(y*ADAS_W+x)*2+1]=(uint8_t)(color>>8);
    }
    render(argv[1],"01_occupied_separate",base,image,lanes,&status);
    status.occupied[0]=0; render(argv[1],"02_same_line_clear",base,second,lanes,&status);
    for(int y=10;y<70;y++)assert(!memcmp(image+(y*ADAS_W+328)*2,second+(y*ADAS_W+328)*2,180*2));
    int occupancy_changed=0;
    for(int y=10;y<70;y++) occupancy_changed|=memcmp(image+((y*ADAS_W+1120)*2),second+((y*ADAS_W+1120)*2),380*2)!=0;
    assert(occupancy_changed);
    struct features f=feature();
    memcpy(image,base,IMAGE_BYTES); memcpy(second,base,IMAGE_BYTES);
    adas_hud(image,&f,NULL,0,lanes,&status,0,"FPS 23.3 | NPU 33.0 MS | OBJECTS 2");
    adas_hud(second,&f,NULL,0,lanes,&status,0,"FPS 55.5 | NPU 33.0 MS | OBJECTS 2");
    int fps_changed=0;
    for(int y=0;y<82;y++) {
        fps_changed|=memcmp(image+(y*ADAS_W+192)*2,second+(y*ADAS_W+192)*2,120*2)!=0;
        assert(!memcmp(image+(y*ADAS_W+328)*2,second+(y*ADAS_W+328)*2,(ADAS_W-328)*2));
    }
    assert(fps_changed); /* FPS changes only its capsule; metrics never repeat it. */

    /* STALE must not change either language of the hazard banner, even at CAUTION. */
    const uint32_t hazards[]={EVENT_PEDESTRIAN,EVENT_APPROACH,EVENT_NEAR};
    for(unsigned i=0;i<sizeof(hazards)/sizeof(hazards[0]);i++)
        for(int level=ADAS_CAUTION;level<=ADAS_DANGER;level++) {
            status=(struct adas_status){.events=hazards[i],.level=(enum adas_level)level,.lane_geometry=1};
            memcpy(image,base,IMAGE_BYTES); memcpy(second,base,IMAGE_BYTES);
            adas_hud(image,&f,NULL,0,lanes,&status,0,"");
            status.events|=EVENT_STALE;
            assert(!strcmp(adas_hud_event_name(&status),i==0?"PED":"NEAR"));
            adas_hud(second,&f,NULL,0,lanes,&status,0,"");
            for(int y=94;y<182;y++)
                assert(!memcmp(image+(y*ADAS_W+24)*2,second+(y*ADAS_W+24)*2,1120*2));
        }
    status=(struct adas_status){.events=EVENT_PEDESTRIAN|EVENT_STALE,.level=ADAS_CAUTION,.lane_geometry=1};
    render(argv[1],"13_stale_person_caution",base,image,lanes,&status);
    status.events=EVENT_RED_MOVING|EVENT_SOLID_FORCE|EVENT_PEDESTRIAN|EVENT_STALE;
    assert(!strcmp(adas_hud_event_name(&status),"RED+MOVE"));

    lanes[0].valid=lanes[1].valid=0; lanes[0].type=LANE_SOLID;
    adas_controls_init(&c,1); c.turn=-1; c.forced_turn=-1; c.forced_until=10;
    scene(&status,lanes,NULL,0,&c,&lamp);
    assert(status.events&EVENT_NO_LANE); assert(!(status.events&(EVENT_SOLID_FORCE|EVENT_TURN_BLOCKED)));
    assert(!strcmp(adas_hud_event_name(&status),"NO LANE"));
    adas_controls_init(&c,0); scene(&status,lanes,NULL,0,&c,&lamp);
    status.display_cnn=1; status.display_class[0]=LANE_CLASS_SOLID; status.display_class[1]=LANE_CLASS_DASHED;
    status.display_held[0]=status.display_held[1]=0;
    render(argv[1],"03_cnn_without_geometry",base,image,lanes,&status);
    assert(!status.lane_geometry&&!lanes[0].valid&&!lanes[1].valid);
    assert(!strcmp(adas_hud_event_name(&status),"NO LANE"));
    assert(!strcmp(adas_hud_lane_value_label(&status,0),"SOLID / NO GEO"));
    assert(!strcmp(adas_hud_lane_value_label(&status,1),"DASHED / NO GEO"));
    assert(!memcmp(image+((450*ADAS_W+960)*2),base+((450*ADAS_W+960)*2),2));
    status.display_held[0]=1; render(argv[1],"04_cnn_hold",base,second,lanes,&status);
    int bottom_changed=0;
    for(int y=1036;y<1068;y++) bottom_changed|=memcmp(image+(y*ADAS_W+40)*2,second+(y*ADAS_W+40)*2,1000*2)!=0;
    assert(bottom_changed);
    status.display_class[0]=status.display_class[1]=LANE_CLASS_UNKNOWN;
    status.display_held[0]=0; render(argv[1],"05_stale_unknown",base,image,lanes,&status);
    status.display_class[0]=LANE_CLASS_NONE; status.display_class[1]=LANE_CLASS_INVALID;
    render(argv[1],"11_cnn_none_invalid",base,image,lanes,&status);

    lanes[0].valid=lanes[1].valid=1;
    adas_controls_init(&c,0); lamp=(struct adas_signal){.candidate=LIGHT_RED,.source=SIGNAL_SEARCH,.stable=1};
    scene(&status,lanes,NULL,0,&c,&lamp);
    assert(status.hud_light==LIGHT_RED&&status.light==LIGHT_UNKNOWN&&!(status.events&EVENT_RED_MOVING));
    assert(!(status.events&EVENT_RED_STOP));
    assert(!strcmp(adas_hud_event_name(&status),"TL WATCH"));
    assert(!strcmp(adas_hud_lane_value_label(&status,0),"SOLID"));
    render(argv[1],"06_search_red",base,image,lanes,&status);
    status.events|=EVENT_PEDESTRIAN; assert(!strcmp(adas_hud_event_name(&status),"PED"));
    lanes[0].valid=lanes[1].valid=0;
    scene(&status,lanes,NULL,0,&c,&lamp);
    assert(!strcmp(adas_hud_event_name(&status),"NO LANE"));
    assert(!(status.events&(EVENT_RED_STOP|EVENT_RED_MOVING)));
    render(argv[1],"06b_search_red_no_geometry",base,image,lanes,&status);
    lamp.source=SIGNAL_ROI; scene(&status,lanes,NULL,0,&c,&lamp);
    assert(status.events&EVENT_RED_STOP); assert(!strcmp(adas_hud_event_name(&status),"RED STOP"));
    render(argv[1],"07_roi_red_stop",base,image,lanes,&status);

    lanes[0].valid=lanes[1].valid=1; lanes[0].type=LANE_SOLID;
    adas_controls_init(&c,1); c.forced_turn=-1; c.forced_until=10;
    scene(&status,lanes,NULL,0,&c,&lamp); assert(status.events&EVENT_SOLID_FORCE);
    assert(status.level==ADAS_DANGER&&!strcmp(adas_hud_event_name(&status),"FORCE"));
    status.phase_time=1./6.; render(argv[1],"08_sim_force",base,image,lanes,&status);
    status.phase_time=.5; render(argv[1],"09_sim_force_dim",base,second,lanes,&status);
    assert(memcmp(image+((4*ADAS_W+4)*2),second+((4*ADAS_W+4)*2),2));
    adas_controls_init(&c,0); lamp=(struct adas_signal){0};
    struct detection person={820,600,1050,1050,.92f,5};
    scene(&status,lanes,&person,1,&c,&lamp); assert(status.level==ADAS_DANGER&&(status.events&EVENT_PEDESTRIAN));
    render(argv[1],"10_visual_person_alert",base,image,lanes,&status);
    adas_controls_init(&c,1); c.speed_known=1; c.speed=60;
    scene(&status,lanes,NULL,0,&c,&lamp);
    assert(status.level==ADAS_CAUTION&&(status.events&EVENT_OVERSPEED));
    render(argv[1],"12_sim_caution",base,image,lanes,&status);
    free(base); free(image); free(second);
    puts("PASS: matching display strips, single FPS, CNN NO GEO labels, NONE/INVALID/stale/hold, unique-result voting, geometry/hold expiry cleanup, reconfirmation, occupancy separation, held FORCE rejection, SEARCH TL WATCH/ROI red, STALE hazard priority in EVT/bilingual banners and danger perimeter");
    return 0;
}
