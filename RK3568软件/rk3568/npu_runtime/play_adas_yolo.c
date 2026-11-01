#define _GNU_SOURCE
#include "adas_core.h"
#include "npu_runtime.h"
#include "adas_hud.h"
#include "adas_nav.h"
#include "adas_signal.h"
#define LIGHT_PICK_MIN_VOTES 45
#define LIGHT_PICK_NEAR_PCT 90
#include "adas_tracker.h"
#include "lane_cnn_v2.h"
#include "lane_pipeline.h"
#include "preprocess.h"
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <math.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <stdatomic.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/wait.h>
#include <time.h>
#include <termios.h>
#include <unistd.h>

/* Display tracks originate in real detections and retain confirmed geometry briefly. */
struct light_disp_mark { int x0,y0,x1,y1; enum adas_light color; int miss; enum adas_light known; unsigned unknown_run; };
struct light_disp_track { struct light_disp_mark previous[MAX_DETECTIONS+1]; int hits[MAX_DETECTIONS+1]; int count; };
static float light_mark_iou(const struct light_disp_mark *a,const struct light_disp_mark *b) {
    float w=fminf(a->x1,b->x1)-fmaxf(a->x0,b->x0)+1.f;
    float h=fminf(a->y1,b->y1)-fmaxf(a->y0,b->y0)+1.f;
    if(w<=0 || h<=0)return 0.f;
    float intersection=w*h;
    float area_a=(a->x1-a->x0+1.f)*(a->y1-a->y0+1.f);
    float area_b=(b->x1-b->x0+1.f)*(b->y1-b->y0+1.f);
    return intersection/(area_a+area_b-intersection);
}
static int light_disp_update(struct light_disp_track *track,struct light_disp_mark *marks,int count) {
    int used_old[MAX_DETECTIONS+1]={0},used_new[MAX_DETECTIONS+1]={0};
    int hits[MAX_DETECTIONS+1]={0};
    struct light_disp_mark observed[MAX_DETECTIONS+1];
    memcpy(observed,marks,(size_t)count*sizeof(*marks));
    for(;;) {
        int old=-1,cur=-1; float best=0.f;
        for(int i=0;i<track->count;i++)if(!used_old[i])
            for(int j=0;j<count;j++)if(!used_new[j]) {
                int compat=track->previous[i].color==marks[j].color||track->previous[i].color==LIGHT_UNKNOWN||marks[j].color==LIGHT_UNKNOWN;
                float iou=light_mark_iou(&track->previous[i],&marks[j]);
                float thr=compat?.3f:.55f;
                if(iou>thr&&iou>best){best=iou;old=i;cur=j;}
            }
        if(old<0)break;
        used_old[old]=used_new[cur]=1; hits[cur]=track->hits[old]+1;
        struct light_disp_mark *m=&marks[cur],*p=&track->previous[old];
        float pcx=(p->x0+p->x1)*.5f,pcy=(p->y0+p->y1)*.5f;
        float pw=p->x1-p->x0,ph=p->y1-p->y0;
        float cx=pcx+fmaxf(-8.f,fminf(8.f,.18f*((m->x0+m->x1)*.5f-pcx)));
        float cy=pcy+fmaxf(-8.f,fminf(8.f,.18f*((m->y0+m->y1)*.5f-pcy)));
        float w=fmaxf(pw*.80f,fminf(pw*1.20f,.20f*(m->x1-m->x0)+.80f*pw));
        float h=fmaxf(ph*.80f,fminf(ph*1.20f,.20f*(m->y1-m->y0)+.80f*ph));
        m->x0=(int)lroundf(cx-w*.5f);m->x1=(int)lroundf(cx+w*.5f);
        m->y0=(int)lroundf(cy-h*.5f);m->y1=(int)lroundf(cy+h*.5f);
        /* Persist last known RED/GREEN across UNKNOWN pixels and short misses. */
        m->unknown_run=m->color==LIGHT_UNKNOWN?p->unknown_run+1:0;
        m->known=p->known;
        if(m->color!=LIGHT_UNKNOWN) m->known=m->color;
        else if(m->known!=LIGHT_UNKNOWN) m->color=m->known;
        else if(p->color!=LIGHT_UNKNOWN){m->color=p->color;m->known=p->color;}
    }
    for(int i=0;i<count;i++)for(int j=0;j<i;j++)
        if(marks[i].color==marks[j].color&&
           abs(marks[i].x0+marks[i].x1-marks[j].x0-marks[j].x1)<=12&&
           abs(marks[i].y0+marks[i].y1-marks[j].y0-marks[j].y1)<=12) {
            marks[i]=observed[i];marks[j]=observed[j];
        }
    for(int i=0;i<count;i++)if(!hits[i]) {
        hits[i]=1;
        for(int j=0;j<track->count;j++)if(!used_old[j]&&track->hits[j]>=2&&
            abs(marks[i].x0+marks[i].x1-track->previous[j].x0-track->previous[j].x1)<=20&&
            abs(marks[i].y0+marks[i].y1-track->previous[j].y0-track->previous[j].y1)<=20) {
            hits[i]=track->hits[j]+1;used_old[j]=1;
            marks[i].unknown_run=marks[i].color==LIGHT_UNKNOWN?track->previous[j].unknown_run+1:0;
            marks[i].known=track->previous[j].known;
            if(marks[i].color!=LIGHT_UNKNOWN) marks[i].known=marks[i].color;
            else if(marks[i].known!=LIGHT_UNKNOWN) marks[i].color=marks[i].known;
            else if(track->previous[j].color!=LIGHT_UNKNOWN){
                marks[i].color=track->previous[j].color;marks[i].known=track->previous[j].color;}
            break;
        }
        marks[i].miss=0;
        if(marks[i].color!=LIGHT_UNKNOWN) marks[i].known=marks[i].color;
        else if(!marks[i].known) marks[i].known=LIGHT_UNKNOWN;
    } else {
        marks[i].miss=0;
        if(marks[i].color!=LIGHT_UNKNOWN) marks[i].known=marks[i].color;
        else if(marks[i].known!=LIGHT_UNKNOWN) marks[i].color=marks[i].known;
    }
    for(int i=0;i<track->count&&count<MAX_DETECTIONS;i++)
        if(!used_old[i]&&track->hits[i]>=2&&track->previous[i].miss<12) {
            int duplicate=0;
            for(int j=0;j<count;j++)if(
                abs(marks[j].x0+marks[j].x1-track->previous[i].x0-track->previous[i].x1)<=24&&
                abs(marks[j].y0+marks[j].y1-track->previous[i].y0-track->previous[i].y1)<=24)duplicate=1;
            if(!duplicate){marks[count]=track->previous[i];marks[count].miss++;hits[count]=track->hits[i];count++;}
        }
    memcpy(track->previous,marks,(size_t)count*sizeof(*marks));
    memcpy(track->hits,hits,(size_t)count*sizeof(*hits));track->count=count;
    int drawn=0;
    /* Do not paint UNKNOWN hollows once we prefer sticky known colors; avoids white↔green flicker. */
    for(int i=0;i<count;i++)if(hits[i]>=2&&marks[i].color!=LIGHT_UNKNOWN)marks[drawn++]=marks[i];
    return drawn;
}

struct video_info { uint32_t width,height,pitch,buffers; uint64_t bytes; };
struct completion { uint32_t previous,sequence,buffer,reserved; };
#define START _IO('A',1)
#define WAIT _IOWR('A',2,struct completion)
#define INFO _IOR('A',3,struct video_info)
#define STOP _IO('A',4)
_Static_assert(sizeof(struct video_info)==24,"INFO ABI");
_Static_assert(sizeof(struct completion)==16,"WAIT ABI");
static volatile sig_atomic_t stopping;
static void stop_signal(int n) { (void)n; stopping=1; }

/* A failed START may already own a grant. Drain it before retrying; never
 * reissue START against an uncertain DMA owner or discard a STOP failure. */
static int start_dma(int fd) {
    for(int attempt=0;attempt<3&&!stopping;attempt++) {
        if(ioctl(fd,START)==0) return attempt?1:0;
        int error=errno;
        if(stopping) return -1;
        if(error!=ETIMEDOUT||attempt==2) { errno=error; return -1; }
        fprintf(stderr,"DMA_RECOVERY START timeout attempt=%d: STOP/drain\n",attempt+1);
        if(ioctl(fd,STOP)<0) { perror("DMA_RECOVERY STOP/drain"); return -1; }
        if(stopping) return -1;
        usleep(20000);
    }
    errno=EINTR; return -1;
}

static double now(void) { struct timespec t; clock_gettime(CLOCK_MONOTONIC,&t); return t.tv_sec+t.tv_nsec/1e9; }
static double thread_cpu(void) { struct timespec t; clock_gettime(CLOCK_THREAD_CPUTIME_ID,&t); return t.tv_sec+t.tv_nsec/1e9; }
struct slot {
    uint8_t *payload;
    struct adas_nav_slot nav;
    struct features f;
    uint32_t sequence;
    double arrived,enqueued,wait_ms,copy_ms;
    int count,light_count;
    struct detection detections[MAX_DETECTIONS],lights[MAX_DETECTIONS];
    double begin,pre_end,infer_end,decode_end,result_enqueued;
    double pre_cpu_ms,input_ms,run_ms,output_ms;
    int busy;
};
struct pipeline {
    struct slot slots[PREP_SOURCE_SLOTS];
    pthread_mutex_t mutex;
    pthread_cond_t available;
    int pending,ready,done,infer_done,parallel;
    atomic_int failed;
    uint64_t captured,dropped,processed,inferred;
    struct npu_runtime npu;
    struct preprocessor prep;
    float confidence,iou;
    double replay_fps;
    int pipe_fd;
    FILE *raw,*json;
    int interactive,debug_hud,speed_limit,raw_boxes,quad_composite,nav_composite,intersection;
    int passthrough,yolo_only,model10,signal_zoom,signal_wide,scenario_left;
    enum quad_layout quad_layout;
    struct signal_roi signal_roi;
    enum adas_nav_direction nav_signal_direction;
    struct lane_cnn_reader cnn;
    int cnn_enabled,cnn_use_type,fpga_keys;
    struct lane_key_reader keys;
    enum road_profile road_profile;
};
/* Nonblocking write keeps shutdown and a stalled display bounded. Never drop
 * part of a raw frame: a timeout terminates the stream before another frame. */
static int send_frame(int fd,const uint8_t *p) {
    size_t remain=IMAGE_BYTES; double deadline=now()+1.;
    while(remain&&!stopping) {
        ssize_t n=write(fd,p,remain);
        if(n>0) { p+=n; remain-=(size_t)n; continue; }
        if(n<0&&errno==EINTR) continue;
        if(n<0&&(errno==EAGAIN||errno==EWOULDBLOCK)) {
            if(now()>deadline) { errno=ETIMEDOUT; return -1; }
            struct pollfd pollfd={fd,POLLOUT,0};
            if(poll(&pollfd,1,50)<0&&errno!=EINTR) return -1;
            if(pollfd.revents&(POLLERR|POLLHUP|POLLNVAL)) { errno=EPIPE; return -1; }
            continue;
        }
        if(n==0) errno=EIO;
        return -1;
    }
    return remain?-1:0;
}
/* Each timed wait is interruptible by SIGTERM and worker failure. */
static void wait_pipeline(struct pipeline *p) {
    struct timespec until; clock_gettime(CLOCK_MONOTONIC,&until);
    until.tv_nsec+=100000000;
    if(until.tv_nsec>=1000000000) { until.tv_sec++; until.tv_nsec-=1000000000; }
    pthread_cond_timedwait(&p->available,&p->mutex,&until);
}
static int infer_slot(struct pipeline *p,int index) {
    struct slot *s=&p->slots[index];
    s->begin=now();
    double cpu=thread_cpu();
    if(prep_run(&p->prep,index)) return -1;
    s->pre_cpu_ms=(thread_cpu()-cpu)*1000;
    s->pre_end=now();
    const float *outputs[9];
    int inferred=npu_infer(&p->npu,p->prep.output.data,outputs,
        &s->input_ms,&s->run_ms,&s->output_ms);
    int input_done=prep_output_end(&p->prep);
    if(inferred||input_done) return -1;
    s->infer_end=now();
    s->count=p->model10?adas_decode10(outputs,p->confidence,p->iou,s->detections):adas_decode(outputs,p->confidence,p->iou,s->detections);
    s->light_count=0;
    /* model10: adas_decode10 maps BDD traffic-light(9)->cls 6; collect lights on 6, not raw 9 */
    if(p->model10&&s->count>=0) {
        int keep=0;
        for(int j=0;j<s->count;j++) {
            if(s->detections[j].cls==6)s->lights[s->light_count++]=s->detections[j];
            else s->detections[keep++]=s->detections[j];
        }
        s->count=keep;
    }
    int released=npu_release(&p->npu);
    if(s->count>=0&&released>=0&&p->model10&&p->intersection&&p->signal_zoom) {
        uint8_t *zoom=malloc(640u*640u*3u);
        if(!zoom)return -1;
        if(prep_cpu_begin(&p->prep,index)){free(zoom);return -1;}
        adas_preprocess_signal_zoom(s->payload,zoom);
        int done=prep_cpu_end(&p->prep,index);
        double input_ms,run_ms,output_ms;
        int rc=done?-1:npu_infer(&p->npu,zoom,outputs,&input_ms,&run_ms,&output_ms);
        if(!rc) {
            struct detection zoomed[MAX_DETECTIONS];
            int num=adas_decode10(outputs,p->confidence,p->iou,zoomed);
            rc=npu_release(&p->npu);
            if(num<0)rc=-1;
            for(int j=0;j<num&&s->light_count<MAX_DETECTIONS;j++)if(zoomed[j].cls==6) {
                zoomed[j].x0=840+zoomed[j].x0/8;zoomed[j].x1=840+zoomed[j].x1/8;
                zoomed[j].y0=360+zoomed[j].y0/8;zoomed[j].y1=360+zoomed[j].y1/8;
                s->lights[s->light_count++]=zoomed[j];
            }
        }
        free(zoom);
        if(rc)return -1;
    }
    if(s->count>=0&&released>=0&&p->model10&&p->intersection&&p->signal_wide) {
        uint8_t *wide=malloc(640u*640u*3u);
        if(!wide)return -1;
        if(prep_cpu_begin(&p->prep,index)){free(wide);return -1;}
        adas_preprocess_signal_wide(s->payload,wide);
        int done=prep_cpu_end(&p->prep,index);
        double input_ms,run_ms,output_ms;
        int rc=done?-1:npu_infer(&p->npu,wide,outputs,&input_ms,&run_ms,&output_ms);
        if(!rc) {
            struct detection zoomed[MAX_DETECTIONS];
            int num=adas_decode10(outputs,p->confidence,p->iou,zoomed);
            rc=npu_release(&p->npu);
            if(num<0)rc=-1;
            for(int j=0;j<num&&s->light_count<MAX_DETECTIONS;j++)if(zoomed[j].cls==6) {
                zoomed[j].x0=480+zoomed[j].x0*.5f;zoomed[j].x1=480+zoomed[j].x1*.5f;
                zoomed[j].y0=180+zoomed[j].y0*.5f;zoomed[j].y1=180+zoomed[j].y1*.5f;
                s->lights[s->light_count++]=zoomed[j];
            }
        }
        free(wide);
        if(rc)return -1;
    }
    if(s->count<0||released<0) {
        fprintf(stderr,"YOLO output decoding/release failed\n"); return -1;
    }
    s->decode_end=now();
    pthread_mutex_lock(&p->mutex); p->inferred++; pthread_mutex_unlock(&p->mutex);
    return 0;
}
/* Only this worker accesses RKNN and the shared preprocessing output.
 * Publish copied detections, never RKNN output pointers, to the CPU worker. */
static void *infer_frames(void *arg) {
    struct pipeline *p=arg;
    while(!stopping&&!atomic_load(&p->failed)) {
        pthread_mutex_lock(&p->mutex);
        while(p->pending<0&&!p->done&&!stopping&&!atomic_load(&p->failed)) wait_pipeline(p);
        if(stopping||atomic_load(&p->failed)||(p->pending<0&&p->done)) {
            pthread_mutex_unlock(&p->mutex); break;
        }
        int index=p->pending; p->pending=-1;
        pthread_mutex_unlock(&p->mutex);
        if(infer_slot(p,index)) { atomic_store(&p->failed,1); break; }
        pthread_mutex_lock(&p->mutex);
        while(p->ready>=0&&!stopping&&!atomic_load(&p->failed)) wait_pipeline(p);
        if(stopping||atomic_load(&p->failed)) { pthread_mutex_unlock(&p->mutex); break; }
        p->slots[index].result_enqueued=now();
        p->ready=index;
        pthread_cond_broadcast(&p->available);
        pthread_mutex_unlock(&p->mutex);
    }
    pthread_mutex_lock(&p->mutex); p->infer_done=1;
    pthread_cond_broadcast(&p->available); pthread_mutex_unlock(&p->mutex);
    return NULL;
}
/* Shortcuts share capture, packet validation, ownership and display with ADAS. */
static void *process_shortcut(void *arg) {
    struct pipeline *p=arg;
    while(!stopping&&!atomic_load(&p->failed)) {
        pthread_mutex_lock(&p->mutex);
        int *queue=p->parallel?&p->ready:&p->pending;
        while(*queue<0&&!(p->parallel?p->infer_done:p->done)&&!stopping&&!atomic_load(&p->failed))
            wait_pipeline(p);
        if(stopping||atomic_load(&p->failed)||(*queue<0&&(p->parallel?p->infer_done:p->done))) {
            pthread_mutex_unlock(&p->mutex);break;
        }
        int index=*queue;*queue=-1;struct slot *s=&p->slots[index];
        pthread_cond_broadcast(&p->available);pthread_mutex_unlock(&p->mutex);
        if(p->yolo_only&&!p->parallel&&infer_slot(p,index)) {atomic_store(&p->failed,1);break;}
        if(prep_cpu_begin(&p->prep,index)) {atomic_store(&p->failed,1);break;}
        if(p->yolo_only)adas_hud_yolo_only(s->payload,s->detections,s->count,s->f.source_id);
        if(p->pipe_fd>=0&&send_frame(p->pipe_fd,s->payload)) {
            if(!stopping){perror("display pipe");atomic_store(&p->failed,1);}break;
        }
        if(p->raw&&fwrite(s->payload,1,IMAGE_BYTES,p->raw)!=IMAGE_BYTES) {perror("output");atomic_store(&p->failed,1);break;}
        if(p->json) {
            fprintf(p->json,"{\"mode\":\"%s\",\"source_frame_id\":%u,\"detections\":[",p->passthrough?"passthrough":"yolo_only",s->f.source_id);
            for(int i=0;p->yolo_only&&i<s->count;i++) {
                struct detection *d=&s->detections[i];
                fprintf(p->json,"%s{\"class\":%d,\"score\":%.8g,\"box\":[%.4f,%.4f,%.4f,%.4f]}",i?",":"",d->cls,d->score,d->x0,d->y0,d->x1,d->y1);
            }
            fputs("]}\n",p->json);
            if(ferror(p->json)){perror("results");atomic_store(&p->failed,1);break;}
        }
        if(prep_cpu_end(&p->prep,index)){atomic_store(&p->failed,1);break;}
        pthread_mutex_lock(&p->mutex);p->processed++;s->busy=0;
        pthread_cond_broadcast(&p->available);pthread_mutex_unlock(&p->mutex);
    }
    return NULL;
}
static void *process_frames(void *arg) {
    struct pipeline *p=arg;
    struct lane_pipeline lane_processor={0};
    struct adas_nav_state navigation={0};
    struct intersection_state junction={0};
    struct intersection_keys junction_keys={0};
    struct adas_state scene={0}; struct adas_controls controls;
    adas_controls_init(&controls,p->interactive?ADAS_CONTROL_SIMULATED:p->fpga_keys?ADAS_CONTROL_PHYSICAL:ADAS_CONTROL_MONITOR); controls.speed_limit=p->speed_limit; controls.road_profile=p->road_profile;
    struct signal_detector signal_detector;
    if(adas_signal_init(&signal_detector)) { atomic_store(&p->failed,1); return NULL; }
    struct adas_box_display *box_display=adas_box_display_create();
    if(!box_display){adas_signal_close(&signal_detector);atomic_store(&p->failed,1);return NULL;}
    uint32_t last_source=0; double last_time=0,started=now(),fps_started=started,fps=0;
    int have_source=0; unsigned fps_frames=0;
    struct light_track light_track={0};
    struct light_disp_track light_disp={0};
    struct adas_tracker *tracker=!p->raw_boxes&&p->json?adas_tracker_create():NULL;
    if(!p->raw_boxes&&p->json&&!tracker) {
        perror("tracker allocation"); adas_box_display_destroy(box_display); adas_signal_close(&signal_detector);
        atomic_store(&p->failed,1); return NULL;
    }
    fprintf(stderr,"BOX_MODE=%s EVERY_DISPLAY_FRAME_YOLO=1\n",p->raw_boxes?"raw":"ema_iou_hysteresis_v1");
    while(!stopping&&!atomic_load(&p->failed)) {
        pthread_mutex_lock(&p->mutex);
        int *queue=p->parallel?&p->ready:&p->pending;
        while(*queue<0&&!(p->parallel?p->infer_done:p->done)&&!stopping&&!atomic_load(&p->failed))
            wait_pipeline(p);
        if(stopping||atomic_load(&p->failed)||(*queue<0&&(p->parallel?p->infer_done:p->done))) {
            pthread_mutex_unlock(&p->mutex); break;
        }
        int index=*queue; *queue=-1;
        struct slot *s=&p->slots[index];
        pthread_cond_broadcast(&p->available);
        pthread_mutex_unlock(&p->mutex);
        if(!p->parallel) {
            if(infer_slot(p,index)) { atomic_store(&p->failed,1); break; }
            s->result_enqueued=now();
        }
        double post_begin=now(),begin=s->begin,pre_end=s->pre_end,infer_end=s->infer_end;
        double decode_end=s->decode_end,pre_cpu_ms=s->pre_cpu_ms;
        double input_ms=s->input_ms,run_ms=s->run_ms,output_ms=s->output_ms;
        struct detection *detections=s->detections;
        int count=s->count;
        if(prep_cpu_begin(&p->prep,index)) { atomic_store(&p->failed,1); break; }
        struct detection shown_storage[MAX_DETECTIONS];
        struct detection *shown=p->raw_boxes?detections:shown_storage;
        struct track_view tracks[MAX_DETECTIONS]={{0}};
        int tracker_reset=0;
        if(tracker) {
            tracker_reset=adas_tracker_update(tracker,s->payload,s->f.source_id,s->arrived,
                detections,count,shown,tracks);
            if(tracker_reset<0) {
                fprintf(stderr,"Invalid tracker input\n"); atomic_store(&p->failed,1); break;
            }
        }
        double track_end=now();
        double track_ms=(track_end-post_begin)*1000;
        int reset=!have_source||(uint32_t)(s->f.source_id-last_source)>15||s->arrived-last_time>.5;
        if(p->cnn_enabled)lane_cnn_poll(&p->cnn,post_begin);
        const struct lane_cnn_snapshot *cnn_for_frame=p->cnn_enabled&&!p->nav_composite?lane_cnn_for_frame(&p->cnn,s->f.source_id,post_begin):NULL;
        int lane_mode=p->nav_composite?2:p->cnn_use_type;
        struct lane_observation observation;
        lane_pipeline_step(&lane_processor,s->payload,&s->f,detections,count,lane_mode,
            cnn_for_frame,post_begin,reset,&observation);
        const struct lane *lanes=observation.lanes;
        unsigned cnn_applied=observation.applied;
        if(p->cnn_enabled&&p->processed%30==0)fprintf(stderr,"CNN fid=%u video_fid=%u age_frames=%u fresh=%d applied=%u left=%s right=%s\n",
            p->cnn.snapshot.frame_id,s->f.source_id,s->f.source_id-p->cnn.snapshot.frame_id,
            lane_cnn_fresh(&p->cnn,s->f.source_id,post_begin),cnn_applied,
            lane_cnn_label(p->cnn.snapshot.cls[0]),lane_cnn_label(p->cnn.snapshot.cls[1]));
        last_source=s->f.source_id; last_time=s->arrived; have_source=1;
        double lane_end=now();
        int junction_command=JUNCTION_NONE;
        if(p->interactive) {
            unsigned char keys[64]; ssize_t length=read(STDIN_FILENO,keys,sizeof(keys));
            for(ssize_t k=0;k<length;k++) {
                if(keys[k]==27) stopping=1;
                else if(p->intersection&&keys[k]>='0'&&keys[k]<='3')junction_command=keys[k]-'0'+1;
                else adas_control_key(&controls,keys[k],lane_end);
            }
        }
        if(p->fpga_keys&&p->intersection) {
            uint32_t seq,left,right;unsigned down;
            int result=intersection_key_snapshot(p->cnn.map,&seq,&left,&right,&down);
            if(result==1) {
                int command=intersection_key_step(&junction_keys,seq,left,right,down,lane_end);
                if(command<0){memset(&junction,0,sizeof(junction));controls.turn=0;}
                else if(command)junction_command=command;
            } else if(result<0) {
                memset(&junction_keys,0,sizeof(junction_keys));memset(&junction,0,sizeof(junction));controls.turn=0;
            }
        } else if(p->fpga_keys) {
            int turn;
            int key_result=lane_key_poll(p->cnn.map,&p->keys,&turn);
            if(key_result<0){memset(&p->keys,0,sizeof(p->keys));controls.turn=0;}
            if(key_result==1) {
                controls.turn=turn;controls.forced_turn=0;controls.forced_until=0;
                fprintf(stderr,"FPGA_KEY turn=%d left=%u right=%u\n",turn,p->keys.left,p->keys.right);
            }
        }
        if(p->intersection&&junction_command) {
            controls.turn=junction_command==JUNCTION_LEFT?-1:junction_command==JUNCTION_RIGHT?1:0;
            controls.forced_turn=0;controls.forced_until=0;
            fprintf(stderr,"INTERSECTION_KEY command=%d (1 stop,2 left,3 straight,4 right)\n",junction_command);
        }
        if(p->nav_composite) {
            adas_nav_update(&navigation,&s->nav.observation,s->f.source_id,s->arrived,reset);
            /* Apply after keys so blank/unconfirmed pixels also clear manual routes.
             * Navigation supplies an advisory route, never a turn command or
             * permission to promote an unassigned SEARCH traffic light. */
            controls.route=adas_nav_current_control(&navigation.route);
            controls.auto_signal_fsm=0;
            controls.light=LIGHT_UNKNOWN;
        }
        struct adas_signal signal_result;
        int selected_light=-1,raw_pick=-1,raw_light_count=s->light_count,draw_light=0;
        int light_drawn=0,display_obs_count=0,amber_present=0;
        struct detection display_obs[MAX_DETECTIONS];
        enum adas_light display_colors[MAX_DETECTIONS];
        int display_votes[MAX_DETECTIONS];
        struct light_disp_mark light_marks[MAX_DETECTIONS+1];
        int light_mark_votes[MAX_DETECTIONS+1];
        enum adas_light shown_color=LIGHT_UNKNOWN;struct candidate shown_box={0};
        enum adas_light raw_light=LIGHT_UNKNOWN,light_colors[MAX_DETECTIONS];
        int red_votes[MAX_DETECTIONS],green_votes[MAX_DETECTIONS],parent[MAX_DETECTIONS];
        if(p->model10) {
            memset(&signal_result,0,sizeof(signal_result));
            signal_result.source=SIGNAL_SEARCH;
            if(reset||s->f.source_id<=1){
                adas_light_track_reset(&light_track);
                memset(&light_disp,0,sizeof(light_disp));
            }
            int kept=0;
            for(int j=0;j<raw_light_count;j++) {
                struct detection d=s->lights[j];
                float cx=(d.x0+d.x1)*.5f,cy=(d.y0+d.y1)*.5f,w=d.x1-d.x0+1,h=d.y1-d.y0+1;
                if(!isfinite(cx)||!isfinite(cy)||!isfinite(w)||!isfinite(h)||cx<ADAS_W*(p->signal_wide?.25f:.36f)||cx>ADAS_W*(p->signal_wide?.75f:.64f)||cy<ADAS_H*.03f||cy>ADAS_H*.58f||w<6||w>ADAS_W*.10f||h<8||h>ADAS_H*.22f)continue;
                int red,green;enum adas_light color=pixel_light_color(s->payload,&d,&red,&green);
                display_obs[display_obs_count]=d;display_colors[display_obs_count]=color;
                display_votes[display_obs_count++]=red>green?red:green;
                if(color==LIGHT_YELLOW){amber_present=1;continue;}
                if(color==LIGHT_UNKNOWN)continue;
                s->lights[kept]=d;light_colors[kept]=color;red_votes[kept]=red;green_votes[kept]=green;parent[kept]=kept;kept++;
            }
            s->light_count=kept;
            for(int i=0;i<kept;i++)for(int j=0;j<i;j++) {
                struct detection *a=&s->lights[i],*b=&s->lights[j];
                float ha=a->y1-a->y0+1,hb=b->y1-b->y0+1,m=fmaxf(ha,hb);
                float gap=fmaxf(0,fmaxf(a->x0-b->x1,b->x0-a->x1));
                if(fabsf((a->y0+a->y1-b->y0-b->y1)*.5f)<=.9f*m&&gap<=2.5f*m&&ha/hb>=.6f&&ha/hb<=1.6f) {
                    int x=i,y=j;while(parent[x]!=x)x=parent[x];while(parent[y]!=y)y=parent[y];parent[x]=y;
                }
            }
            if(p->scenario_left) {
                int best_v=0;
                for(int i=0;i<kept;i++)if(red_votes[i]+green_votes[i]>best_v)best_v=red_votes[i]+green_votes[i];
                if(best_v>=LIGHT_PICK_MIN_VOTES) {
                    int near_cnt=0,near_left=-1,top=-1;float near_left_cx=1e9f;
                    for(int i=0;i<kept;i++) {
                        int v=red_votes[i]+green_votes[i];
                        if(v*100<best_v*LIGHT_PICK_NEAR_PCT)continue;
                        float cx=(s->lights[i].x0+s->lights[i].x1)*.5f;
                        if(v==best_v&&top<0)top=i;
                        if(cx<near_left_cx){near_left_cx=cx;near_left=i;}
                        near_cnt++;
                    }
                    selected_light=near_cnt>1&&near_left>=0?near_left:top;
                }
            } else {
            int group=-1;float best_group=0;
            for(int i=0;i<kept;i++)if(parent[i]==i) {
                float left=ADAS_W,right=0;
                for(int j=0;j<kept;j++){int q=j;while(parent[q]!=q)q=parent[q];if(q==i){left=fminf(left,s->lights[j].x0);right=fmaxf(right,s->lights[j].x1);}}
                float value=p->scenario_left?left:fabsf((left+right)*.5f-ADAS_W*.5f);
                if(group<0||value<best_group){group=i;best_group=value;}
            }
            if(group>=0) {
                float left=ADAS_W,right=0,best=-1,second=-1;int runner=-1;
                for(int j=0;j<kept;j++){int q=j;while(parent[q]!=q)q=parent[q];if(q!=group)continue;
                    struct detection *d=&s->lights[j];left=fminf(left,d->x0);right=fmaxf(right,d->x1);
                    float area=(d->x1-d->x0+1)*(d->y1-d->y0+1),score=(red_votes[j]+green_votes[j])/area;
                    if(score>best){second=best;runner=selected_light;best=score;selected_light=j;}else if(score>second){second=score;runner=j;}
                }
                if(runner>=0&&best<1.05f*second) {
                    selected_light=-1;
                }
            }
            }
            raw_pick=selected_light;
            if(raw_pick>=0)raw_light=light_colors[raw_pick];
            light_track.allow_switch=p->scenario_left;
            draw_light=adas_light_track_step(&light_track,s->lights,light_colors,red_votes,green_votes,NULL,
                kept,raw_pick,0,s->f.source_id,&shown_color,&shown_box);
            selected_light=light_track.selected>=0&&light_track.selected<kept?light_track.selected:-1;
            if(amber_present)draw_light=0;
            if(draw_light) { signal_result.candidate=shown_color;signal_result.box=shown_box; }

        } else {
            adas_signal_detect(&signal_detector,s->payload,&s->f,detections,count,&p->signal_roi,s->arrived,&signal_result);
            if(p->nav_composite) {
                adas_hud_nav_bind_signal(&signal_result,&navigation.route,p->nav_signal_direction);
                struct adas_signal arrow_signal;
                adas_nav_pixel_signal_with_objects(&navigation,s->payload,detections,count,s->arrived,&arrow_signal);
                if(arrow_signal.stable)signal_result=arrow_signal;
            }
        }
        double signal_end=now();
        struct adas_status status;
        struct features scene_features=s->f;
        if(!p->quad_composite)scene_features.candidates[2].valid=adas_crosswalk_confirm(s->payload,&s->f.candidates[2]);
        adas_state_update(&scene,&scene_features,detections,count,lanes,&controls,&signal_result,s->arrived,signal_end,&status);
        if(p->model10) {
            static enum adas_light hud_hold=LIGHT_UNKNOWN; static int hud_hold_miss=0;
            if(reset||s->f.source_id<=1||amber_present){hud_hold=LIGHT_UNKNOWN;hud_hold_miss=0;}
            if(draw_light&&shown_color!=LIGHT_UNKNOWN){hud_hold=shown_color;hud_hold_miss=0;status.hud_light=shown_color;}
            else if(hud_hold!=LIGHT_UNKNOWN&&hud_hold_miss<12){hud_hold_miss++;status.hud_light=hud_hold;}
            else {status.hud_light=draw_light?shown_color:LIGHT_UNKNOWN; if(!draw_light){hud_hold=LIGHT_UNKNOWN;hud_hold_miss=0;}}
        }
        lane_pipeline_status(&lane_processor,&observation,cnn_for_frame,lane_mode,post_begin,&status);
        adas_hud_prepare(&scene,&scene_features,lanes,&controls,count,&status);
        struct intersection_view junction_view={0};
        if(p->intersection) {
            if(reset)memset(&junction,0,sizeof(junction));
            intersection_update(&junction,junction_command,detections,count,lanes,&status,signal_end,&junction_view);
            intersection_present(&junction_view,&status);
        }
        double scene_end=now();
        char timing[160];
        snprintf(timing,sizeof(timing),"FPS %.1f | NPU %.1f MS | OBJECTS %d",fps,run_ms,count);
        struct detection display_boxes[MAX_DETECTIONS];int display_indices[MAX_DETECTIONS],display_count=count;
        struct adas_status visible=status;
        if(!p->raw_boxes) {
            display_count=adas_box_display_update(box_display,s->payload,s->f.source_id,s->arrived,p->quad_composite?1+(int)p->quad_layout:0,
                detections,count,display_boxes,display_indices);
            if(display_count<0){atomic_store(&p->failed,1);break;}
            visible.lead_index=-1;
            for(int i=0;i<display_count;i++)if(display_indices[i]==status.lead_index&&status.lead_index>=0)visible.lead_index=i;
        }
        double hud_begin=now();
        if(p->nav_composite)adas_hud_nav_reference(s->payload,&navigation.route,navigation.ar_direction,&visible,p->raw_boxes?shown:display_boxes,display_count);
        if(p->intersection)adas_hud_intersection(s->payload,&s->f,p->raw_boxes?shown:display_boxes,
            display_count,lanes,&visible,&junction_view,p->debug_hud,timing);
        else adas_hud_tracked(s->payload,&s->f,p->raw_boxes?shown:display_boxes,display_count,lanes,&visible,p->debug_hud,timing,NULL);
        if(p->model10) {
            int kept_draw=display_obs_count;
            for(int j=0;j<kept_draw;j++) {
                struct light_disp_mark mark={(int)display_obs[j].x0,(int)display_obs[j].y0,(int)display_obs[j].x1,(int)display_obs[j].y1,display_colors[j],0,display_colors[j]==LIGHT_UNKNOWN?LIGHT_UNKNOWN:display_colors[j],0};
                if(mark.x0>mark.x1||mark.y0>mark.y1)continue;
                int votes=display_votes[j],dup=-1;
                for(int k=0;k<light_drawn;k++) {
                    float dx=(mark.x0+mark.x1-light_marks[k].x0-light_marks[k].x1)*.5f;
                    float dy=(mark.y0+mark.y1-light_marks[k].y0-light_marks[k].y1)*.5f;
                    if((mark.color==light_marks[k].color||mark.color==LIGHT_UNKNOWN||light_marks[k].color==LIGHT_UNKNOWN)&&fabsf(dx)<=10.f&&fabsf(dy)<=10.f){dup=k;break;}
                }
                if(dup<0){light_marks[light_drawn]=mark;light_mark_votes[light_drawn++]=votes;}
                else {
                    if(votes>light_mark_votes[dup]){light_marks[dup]=mark;light_mark_votes[dup]=votes;}
                }
            }
        }
        if(p->model10) {
            light_drawn=light_disp_update(&light_disp,light_marks,light_drawn);
            int live=0;
            for(int k=0;k<light_drawn;k++) {
                int nearby=0,same=0,unknown=0,conflict=0;
                for(int j=0;j<display_obs_count;j++) {
                    float dx=(display_obs[j].x0+display_obs[j].x1-light_marks[k].x0-light_marks[k].x1)*.5f;
                    float dy=(display_obs[j].y0+display_obs[j].y1-light_marks[k].y0-light_marks[k].y1)*.5f;
                    if(fabsf(dx)>12.f||fabsf(dy)>12.f)continue;
                    nearby=1;
                    if(display_colors[j]==light_marks[k].color)same=1;
                    else if(display_colors[j]==LIGHT_UNKNOWN)unknown=1;
                    else conflict=1;
                }
                /* A short visual hold never overrides a current different color. */
                if(amber_present&&light_marks[k].color!=LIGHT_YELLOW)continue;
                if(conflict)continue;
                if(same||(unknown&&light_marks[k].unknown_run<=2)||
                   (!nearby&&light_marks[k].miss>0&&light_marks[k].miss<=2))
                    light_marks[live++]=light_marks[k];
            }
            light_drawn=live;
        }
        for(int k=0;k<light_drawn;k++) {
            if(light_marks[k].color==LIGHT_UNKNOWN) continue;
            adas_hud_pixel_light_mark(s->payload,light_marks[k].x0,light_marks[k].y0,
                light_marks[k].x1,light_marks[k].y1,light_marks[k].color,0.0f,p->intersection);
        }
        if(!p->nav_composite&&(p->cnn_enabled||p->cnn_use_type==2)) {
            char cnn_status[160];
            if(cnn_for_frame)snprintf(cnn_status,sizeof(cnn_status),
                "SYNC | F %u | GAP %u/%u | FUSED %u",
                cnn_for_frame->frame_id,cnn_for_frame->confidence[0],cnn_for_frame->confidence[1],cnn_applied);
            else snprintf(cnn_status,sizeof(cnn_status),"WAIT SYNC | VIDEO F %u | BAR F %u",
                s->f.source_id,p->cnn.snapshot.frame_id);
            if(p->cnn_use_type==2)snprintf(cnn_status,sizeof(cnn_status),"F %u | L %s R %s | ROI HITS %d/%d",
                s->f.source_id,adas_lane_source_name(status.lane_source[0]),adas_lane_source_name(status.lane_source[1]),
                status.roi_hits[0],status.roi_hits[1]);
            adas_hud_cnn(s->payload,&status,cnn_status,cnn_for_frame!=NULL||observation.guided[0]||observation.guided[1]);
        }
        uint8_t *presented_image=s->payload;
        if(p->nav_composite) {
            adas_nav_compose(&s->nav,s->payload);
            presented_image=s->nav.original;
            adas_hud_nav_card(presented_image,&navigation.route,&visible);
        }
        adas_hud_return_button(presented_image);
        double draw_end=now();
        if(p->pipe_fd>=0&&send_frame(p->pipe_fd,presented_image)) {
            if(!stopping) { perror("display pipe"); atomic_store(&p->failed,1); } break;
        }
        if(p->raw&&fwrite(presented_image,1,IMAGE_BYTES,p->raw)!=IMAGE_BYTES) { perror("output"); atomic_store(&p->failed,1); break; }
        double sent=now(); uint64_t captured,dropped,inferred;
        fps_frames++;
        if(sent-fps_started>=1.) { fps=fps_frames/(sent-fps_started); fps_frames=0; fps_started=sent; }
        pthread_mutex_lock(&p->mutex);
        p->processed++; captured=p->captured; dropped=p->dropped; inferred=p->inferred;
        pthread_mutex_unlock(&p->mutex);
        if(p->json) {
            fputc('{',p->json);
            if(p->model10) {
                fprintf(p->json,"\"traffic_light_candidates\":%d,\"light_box_count\":%d,\"pixel_light\":%d,\"pixel_light_raw\":%d,\"light_pick\":%d,\"light_pick_raw\":%d,\"light_hold\":%u,\"light_box\":[%u,%u,%u,%u],\"light_side\":\"%s\",\"light_boxes\":[",
                    raw_light_count,light_drawn,signal_result.candidate,raw_light,selected_light,raw_pick,light_track.hold,shown_box.x0,shown_box.y0,shown_box.x1,shown_box.y1,draw_light?(s->light_count>1&&p->scenario_left?"left":"straight"):"none");
                for(int j=0;j<s->light_count;j++)fprintf(p->json,"%s[%.1f,%.1f,%.1f,%.1f]",j?",":"",s->lights[j].x0,s->lights[j].y0,s->lights[j].x1,s->lights[j].y1);
                fputs("],\"light_votes\":[",p->json);
                for(int j=0;j<s->light_count;j++)fprintf(p->json,"%s[%d,%d]",j?",":"",red_votes[j],green_votes[j]);
                fputs("],\"light_display_observations\":[",p->json);
                for(int j=0;j<display_obs_count;j++)fprintf(p->json,"%s[%.1f,%.1f,%.1f,%.1f,%d]",j?",":"",display_obs[j].x0,display_obs[j].y0,display_obs[j].x1,display_obs[j].y1,(int)display_colors[j]);
                fputs("],\"light_drawn_boxes\":[",p->json);
                for(int k=0;k<light_drawn;k++)fprintf(p->json,"%s[%d,%d,%d,%d,%d,%d]",k?",":"",light_marks[k].x0,light_marks[k].y0,light_marks[k].x1,light_marks[k].y1,(int)light_marks[k].color,light_marks[k].miss);
                fputs("],",p->json);
            }
            if(p->intersection)fprintf(p->json,"\"intersection\":{\"source\":\"live_perception\",\"command\":%d,\"motion_is_key_demo\":true,\"red_alarm\":%d,\"pedestrian_alarm\":%d,\"pedestrian_present\":%d,\"permission\":[%d,%d,%d]},",
                junction_view.command,junction_view.red_alarm,junction_view.pedestrian_alarm,junction_view.pedestrian_present,
                junction_view.permission[0],junction_view.permission[1],junction_view.permission[2]);
            if(p->nav_composite)adas_nav_write_json(p->json,&s->nav.observation,&navigation.route);
            fprintf(p->json,"\"hud_ms\":%.4f,",(draw_end-hud_begin)*1000);
            char hud_json[4096];
            if(adas_hud_snapshot(&status,p->quad_composite,hud_json,sizeof(hud_json))) {atomic_store(&p->failed,1);break;}
            fprintf(p->json,"\"hud_presentation\":%s,",hud_json);
            if(p->cnn_enabled&&cnn_for_frame)fprintf(p->json,"\"cnn_used_frame_id\":%u,",cnn_for_frame->frame_id);
            else if(p->cnn_enabled)fprintf(p->json,"\"cnn_used_frame_id\":null,");
            fprintf(p->json,"\"lane_display\":{\"source\":\"%s\",\"classes\":[\"%s\",\"%s\"],\"held\":[%d,%d],\"sample_frame\":[%u,%u],\"age_ms\":[%.4f,%.4f],\"occupied\":[%d,%d],\"occupancy_known\":%d},\"hud_event\":\"%s\",\"hud_light\":%d,",
                status.display_cnn?"CNN":"RULES",adas_lane_class_name(status.display_class[0]),adas_lane_class_name(status.display_class[1]),
                status.display_held[0],status.display_held[1],status.display_frame[0],status.display_frame[1],
                status.display_age_ms[0],status.display_age_ms[1],status.occupied[0],status.occupied[1],status.lane_geometry,
                adas_hud_event_name(&status),status.hud_light);
            fprintf(p->json,"\"competition\":{\"ego_lane\":%d,\"search_link\":%d,\"coasting\":[%d,%d],\"gap_veto\":[%d,%d]},",
                status.ego_lane,status.auto_signal_fsm,status.coasting_valid[0],status.coasting_valid[1],status.gap_veto[0],status.gap_veto[1]);
            char offset_json[40]="null",growth_json[40]="null";
            if(status.lane_geometry)snprintf(offset_json,sizeof(offset_json),"%.4f",status.lane_center_norm);
            if(status.approach_known)snprintf(growth_json,sizeof(growth_json),"%.4f",status.approach_rate);
            fprintf(p->json,"\"road\":{\"scene\":\"%s\",\"action\":\"%s\",\"profile_config\":%d,\"vehicles\":%d,\"vru\":%d,\"image_lane_offset\":%s,\"image_box_growth_per_s\":%s,\"cut_in_side\":%d,\"rear_coverage\":false,\"physical_range_known\":false,\"motion\":",
                adas_road_scene_name(status.road_scene),adas_road_action_name(status.road_action),status.road_profile,
                status.vehicle_count,status.vru_count,offset_json,growth_json,status.cut_in_side);
            if(status.motion_known)fprintf(p->json,"%d",status.moving);else fputs("null",p->json);
            fputs(",\"speed_kmh\":",p->json);if(status.speed_known)fprintf(p->json,"%d",status.speed);else fputs("null",p->json);
            fprintf(p->json,"},\"roi\":{\"source\":[\"%s\",\"%s\"],\"fixed_hits\":[%d,%d],\"guided\":[%d,%d],\"guided_top\":[%d,%d],\"bar_model_version\":%u,\"bar_roi_version\":%u},",
                adas_lane_source_name(status.lane_source[0]),adas_lane_source_name(status.lane_source[1]),
                status.roi_hits[0],status.roi_hits[1],status.roi_guided[0],status.roi_guided[1],status.roi_top[0],status.roi_top[1],status.cnn_model_version,status.cnn_roi_version);
            if(p->cnn_enabled)fprintf(p->json,"\"cnn_frame_id\":%u,\"cnn_sequence\":%u,\"cnn_applied\":%u,\"cnn_left\":\"%s\",\"cnn_right\":\"%s\",",
                p->cnn.snapshot.frame_id,p->cnn.snapshot.sequence,cnn_applied,
                lane_cnn_label(p->cnn.snapshot.cls[0]),lane_cnn_label(p->cnn.snapshot.cls[1]));
            fprintf(p->json,"\"pipeline_mode\":\"%s\",\"inferred\":%"PRIu64",\"result_queue_ms\":%.4f,\"process_ms\":%.4f,",
                p->parallel?"parallel":"serial",inferred,(post_begin-s->result_enqueued)*1000,(sent-post_begin)*1000);
            fprintf(p->json,"\"preprocess_backend\":\"%s\",\"pre_cpu_ms\":%.4f,",p->prep.rga?"rga":"cpu",pre_cpu_ms);
            fprintf(p->json,"\"source_frame_id\":%u,\"sequence\":%u,\"captured\":%"PRIu64",\"dropped\":%"PRIu64",\"detections\":[",
                s->f.source_id,s->sequence,captured,dropped);
            for(int i=0;i<count;i++) fprintf(p->json,"%s{\"class\":%d,\"name\":\"%s\",\"score\":%.8g,\"box\":[%.4f,%.4f,%.4f,%.4f]}",
                i?",":"",detections[i].cls,adas_class_name(detections[i].cls),detections[i].score,
                detections[i].x0,detections[i].y0,detections[i].x1,detections[i].y1);
            fprintf(p->json,"],\"box_mode\":\"%s\",\"track_telemetry_mode\":\"%s\",\"tracker_reset\":%d,\"track_ms\":%.4f,\"tracks\":[",
                p->raw_boxes?"raw":"ema_iou_hysteresis_v1",p->raw_boxes?"raw":"ema_iou_telemetry_v2",tracker_reset,track_ms);
            for(int i=0;i<count;i++) fprintf(p->json,
                "%s{\"raw_index\":%d,\"id\":%"PRIu64",\"confirmed\":%u,\"hits\":%u,\"box\":[%.4f,%.4f,%.4f,%.4f],\"velocity_px_s\":[%.4f,%.4f]}",
                i?",":"",i,tracks[i].id,tracks[i].confirmed,tracks[i].hits,
                shown[i].x0,shown[i].y0,shown[i].x1,shown[i].y1,tracks[i].vx,tracks[i].vy);
            fprintf(p->json,"],\"adas\":{\"level\":\"%s\",\"events\":%u,\"lane_hints\":[\"%s\",\"%s\"],\"signal_candidate\":%d,\"signal_source\":%d,\"signal_stable\":%d,\"assigned_light\":%d,\"simulated_controls\":%d,\"turn\":%d,\"moving\":%d,\"motion_known\":%d,\"speed\":%d,\"speed_known\":%d,\"speed_limit\":%d,\"route\":%d,\"signal_ms\":%.4f,\"fsm_ms\":%.4f},",
                adas_level_name(status.level),status.events,adas_hint_name(status.hints[0]),adas_hint_name(status.hints[1]),
                status.signal.candidate,status.signal.source,status.signal.stable,status.light,status.simulated,status.turn,status.moving,
                status.motion_known,status.speed,status.speed_known,status.speed_limit,status.route,(signal_end-lane_end)*1000,(scene_end-signal_end)*1000);
            fprintf(p->json,"\"processed_fps\":%.3f,\"lane_geometry\":%d,\"signal_confidence\":%.4f,\"lane_measurements\":[{\"valid\":%u,\"coverage\":%.4f,\"residual\":%.4f},{\"valid\":%u,\"coverage\":%.4f,\"residual\":%.4f}],",
                fps,status.lane_geometry,status.signal.confidence,lanes[0].valid,lanes[0].coverage,lanes[0].residual,
                lanes[1].valid,lanes[1].coverage,lanes[1].residual);
            fprintf(p->json,"\"lanes\":[%d,%d],\"wait_ms\":%.4f,\"copy_check_ms\":%.4f,\"queue_ms\":%.4f,\"pre_ms\":%.4f,\"input_ms\":%.4f,\"npu_ms\":%.4f,\"output_ms\":%.4f,\"post_ms\":%.4f,\"lane_ms\":%.4f,\"draw_ms\":%.4f,\"pipe_ms\":%.4f,\"host_age_ms\":%.4f}\n",
                lanes[0].type,lanes[1].type,s->wait_ms,s->copy_ms,(begin-s->enqueued)*1000,(pre_end-begin)*1000,input_ms,run_ms,output_ms,
                (decode_end-infer_end)*1000,(lane_end-track_end)*1000,(draw_end-scene_end)*1000,(sent-draw_end)*1000,(sent-s->arrived)*1000);
            if(fflush(p->json)) { perror("results"); atomic_store(&p->failed,1); break; }
        }
        if(p->processed==1||p->processed%30==0) {
            printf("processed=%"PRIu64" captured=%"PRIu64" dropped=%"PRIu64" source=%u sequence=%u det=%d fps=%.2f host_age_ms=%.2f npu_ms=%.2f track_ms=%.2f pipe_ms=%.2f\n",
                p->processed,captured,dropped,s->f.source_id,s->sequence,count,p->processed/(sent-started),(sent-s->arrived)*1000,run_ms,track_ms,(sent-draw_end)*1000);
            fflush(stdout);
        }
        /* Slot stays owned until all telemetry reads finish. */
        if(prep_cpu_end(&p->prep,index)) { atomic_store(&p->failed,1); break; }
        pthread_mutex_lock(&p->mutex); s->busy=0; pthread_cond_broadcast(&p->available); pthread_mutex_unlock(&p->mutex);
    }
    adas_nav_close(&navigation); adas_box_display_destroy(box_display); adas_tracker_destroy(tracker); adas_signal_close(&signal_detector); return NULL;
}
static pid_t start_display(int *write_fd,int connector) {
    int fds[2];
    if(pipe2(fds,O_CLOEXEC)) return -1;
    if(fcntl(fds[1],F_SETPIPE_SZ,1048576)<0) perror("pipe capacity (default retained)");
    pid_t child=fork();
    if(child==0) {
        if(dup2(fds[0],0)<0) _exit(126);
        close(fds[0]); close(fds[1]);
        char property[64]; snprintf(property,sizeof(property),"connector-id=%d",connector);
        char *gst[]={"gst-launch-1.0","-q","fdsrc","fd=0","blocksize=4147200","!",
            "rawvideoparse","width=1920","height=1080","format=rgb16","framerate=30/1","!",
            "queue","max-size-buffers=1","max-size-bytes=0","max-size-time=0","leaky=downstream","!",
            "kmssink","sync=false","skip-vsync=true",connector<0?NULL:property,NULL};
        execvp(gst[0],gst); perror("gst-launch-1.0"); _exit(127);
    }
    close(fds[0]);
    if(child<0) { close(fds[1]); return -1; }
    if(fcntl(fds[1],F_SETFL,fcntl(fds[1],F_GETFL)|O_NONBLOCK)<0) { close(fds[1]); kill(child,SIGTERM); waitpid(child,NULL,0); return -1; }
    *write_fd=fds[1]; return child;
}
static int finish_display(pid_t child) {
    if(child<0) return 0;
    int status=0; pid_t ret=0;
    for(int stage=0;stage<3;stage++) {
        double deadline=now()+(stage==0?2.:1.);
        do { ret=waitpid(child,&status,WNOHANG); if(ret<0&&errno==EINTR) ret=0; if(ret==0) usleep(10000); } while(ret==0&&now()<deadline);
        if(ret!=0) break;
        kill(child,stage==0?SIGTERM:SIGKILL);
    }
    if(ret==0) do { ret=waitpid(child,&status,0); } while(ret<0&&errno==EINTR);
    return ret<0||!WIFEXITED(status)||WEXITSTATUS(status)?-1:0;
}
static unsigned long number(const char *arg,int *bad) {
    char *end; errno=0; unsigned long n=strtoul(arg,&end,10);
    if(errno||end==arg||*end||arg[0]=='-') *bad=1;
    return n;
}
int main(int argc,char **argv) {
    const char *device="/dev/adas_video0",*model="models/best_rk3568_int8_rkopt.rknn",*library="vendor/librknnrt.so";
    const char *cnn_bar=NULL;
    const char *replay=NULL,*output=NULL,*results=NULL; int headless=0,connector=-1,bad=0,allow_test=0,fast_crc=0;
    const char *preprocess="cpu",*rga_library="vendor/librga/librga.so",*rga_heap=NULL;
    unsigned long limit=0;
    struct pipeline p={.pending=-1,.ready=-1,.parallel=1,.pipe_fd=-1,.confidence=.25f,.iou=.7f,.speed_limit=50};
    const char *hud_mode=getenv("ADAS_HUD_MODE");
    p.nav_composite=hud_mode&&!strcmp(hud_mode,"nav");
    p.intersection=hud_mode&&!strcmp(hud_mode,"intersection");
    p.signal_roi.intersection=p.intersection;
    for(int i=1;i<argc;i++) {
        if(!strcmp(argv[i],"--signal-zoom")) p.signal_zoom=1;
        else if(!strcmp(argv[i],"--signal-wide")) p.signal_wide=1;
        else if(!strcmp(argv[i],"--model10")) p.model10=1;
        else if(!strcmp(argv[i],"--scenario")&&i+1<argc) {
            const char *scenario=argv[++i];
            if(!strcmp(scenario,"left_turn"))p.scenario_left=1;
            else if(!strcmp(scenario,"red_wait"))p.scenario_left=0;
            else bad=1;
        }
        else if(!strcmp(argv[i],"--headless")) headless=1;
        else if(!strcmp(argv[i],"--mode")&&i+1<argc) {
            const char *mode=argv[++i];
            if(!strcmp(mode,"passthrough"))p.passthrough=1;
            else if(!strcmp(mode,"yolo_only"))p.yolo_only=1;
            else bad=1;
        }
        else if(!strcmp(argv[i],"--preprocess")&&i+1<argc) preprocess=argv[++i];
        else if(!strcmp(argv[i],"--rga-library")&&i+1<argc) rga_library=argv[++i];
        else if(!strcmp(argv[i],"--rga-heap")&&i+1<argc) rga_heap=argv[++i];
        else if(!strcmp(argv[i],"--interactive")) p.interactive=1;
        else if(!strcmp(argv[i],"--fpga-keys")) p.fpga_keys=1;
        else if(!strcmp(argv[i],"--fast-crc")) fast_crc=1;
        else if(!strcmp(argv[i],"--quad-composite")) p.quad_composite=1;
        else if(!strcmp(argv[i],"--nav-composite")) p.nav_composite=1;
        else if(!strcmp(argv[i],"--quad-layout")&&i+1<argc) {
            const char *layout=argv[++i];
            if(!strcmp(layout,"rear-quarter"))p.quad_layout=QUAD_LAYOUT_REAR_QUARTER;
            else if(!strcmp(layout,"legacy"))p.quad_layout=QUAD_LAYOUT_LEGACY;
            else bad=1;
        }
        else if(!strcmp(argv[i],"--cnn-bar")&&i+1<argc) cnn_bar=argv[++i];
        else if(!strcmp(argv[i],"--lane-mode")&&i+1<argc) { const char *mode=argv[++i]; if(!strcmp(mode,"cnn"))p.cnn_use_type=1; else if(!strcmp(mode,"adaptive"))p.cnn_use_type=2; else if(!strcmp(mode,"rules"))p.cnn_use_type=0; else bad=1; }
        else if(!strcmp(argv[i],"--road-profile")&&i+1<argc) {
            const char *profile=argv[++i];
            if(!strcmp(profile,"auto"))p.road_profile=ROAD_AUTO;
            else if(!strcmp(profile,"urban"))p.road_profile=ROAD_URBAN;
            else if(!strcmp(profile,"highway"))p.road_profile=ROAD_HIGHWAY;
            else bad=1;
        }
        else if(!strcmp(argv[i],"--debug-hud")) p.debug_hud=1;
        else if(!strcmp(argv[i],"--serial-processing")) p.parallel=0;
        else if(!strcmp(argv[i],"--raw-boxes")) p.raw_boxes=1;
        else if(!strcmp(argv[i],"--speed-limit")&&i+1<argc) { unsigned long v=number(argv[++i],&bad); if(v<10||v>150) bad=1; else p.speed_limit=(int)v; }
        else if(!strcmp(argv[i],"--signal-roi")&&i+1<argc) {
            struct signal_roi *r=&p.signal_roi; char extra;
            if(sscanf(argv[++i],"%d,%d,%d,%d%c",&r->x,&r->y,&r->w,&r->h,&extra)!=4||
               r->x<0||r->x>=ADAS_W||r->y<0||r->y>=ADAS_H||r->w<4||r->w>ADAS_W-r->x||r->h<4||r->h>ADAS_H-r->y) bad=1;
            else r->enabled=1;
        }
        else if(!strcmp(argv[i],"--allow-internal-test")) allow_test=1;
        else if(!strcmp(argv[i],"--frames")&&i+1<argc) limit=number(argv[++i],&bad);
        else if(!strcmp(argv[i],"--connector-id")&&i+1<argc) { unsigned long n=number(argv[++i],&bad); if(n>INT32_MAX) bad=1; else connector=(int)n; }
        else if(!strcmp(argv[i],"--device")&&i+1<argc) device=argv[++i];
        else if(!strcmp(argv[i],"--model")&&i+1<argc) model=argv[++i];
        else if(!strcmp(argv[i],"--library")&&i+1<argc) library=argv[++i];
        else if(!strcmp(argv[i],"--replay")&&i+1<argc) replay=argv[++i];
        else if(!strcmp(argv[i],"--replay-fps")&&i+1<argc) {
            char *end; const char *arg=argv[++i]; errno=0; double value=strtod(arg,&end);
            if(errno||end==arg||*end||!isfinite(value)||value<=0||value>240) bad=1;
            else p.replay_fps=value;
        }
        else if(!strcmp(argv[i],"--output")&&i+1<argc) output=argv[++i];
        else if(!strcmp(argv[i],"--results")&&i+1<argc) results=argv[++i];
        else if((!strcmp(argv[i],"--conf")||!strcmp(argv[i],"--iou"))&&i+1<argc) {
            int conf=!strcmp(argv[i],"--conf"); char *end; errno=0; const char *s=argv[++i]; float v=strtof(s,&end);
            if(errno||s==end||*end||!(v>0&&v<=1)) bad=1;
            else if(conf) p.confidence=v; else p.iou=v;
        } else {
            fprintf(stderr,"快捷模式：--mode passthrough|yolo_only\n");
            fprintf(stderr,"用法：%s [--headless] [--quad-composite] [--frames CAPTURE_COUNT] [--connector-id N] [--device PATH] [--model PATH] [--library PATH] [--replay RGB565_META_STREAM] [--replay-fps FPS] [--output RGB565_STREAM] [--results JSONL] [--conf .25] [--iou .7] [--allow-internal-test] [--interactive] [--fpga-keys] [--fast-crc] [--debug-hud] [--raw-boxes] [--serial-processing] [--speed-limit 50] [--signal-roi X,Y,W,H] [--cnn-bar RESOURCE0] [--lane-mode rules|cnn|adaptive] [--road-profile auto|urban|highway] [--preprocess cpu|rga] [--rga-library PATH] [--rga-heap PATH]\n",argv[0]);
            fprintf(stderr,"--quad-layout legacy|rear-quarter：新布局须配合 --quad-composite；ADAS_HUD_MODE=cruise 启用巡航显示。\n");
            fprintf(stderr,"--nav-composite 或 ADAS_HUD_MODE=nav：原生像素导航，保留地图，仅道路 ROI 参与感知。\n");
            fprintf(stderr,"--interactive is lab-only；仅限显式交互演示，Q/E/R/W 可模拟强制变道、灯态和运动。启用前须设置 ADAS_ALLOW_INTERACTIVE=1。\n");
            return !strcmp(argv[i],"--help")?0:2;
        }
    }
    if(p.model10&&access(model,R_OK)!=0) {fprintf(stderr,"MODEL10_MISSING: %s\n",model);return 2;}
    if(p.model10&&!p.intersection) {fprintf(stderr,"--model10 requires ADAS_HUD_MODE=intersection\n");return 2;}
    if((p.signal_zoom||p.signal_wide)&&(!p.model10||!p.intersection)) {fprintf(stderr,"--signal-zoom requires --model10 and intersection mode\n");return 2;}
    if(p.passthrough||p.yolo_only) {
        if((p.passthrough&&p.yolo_only)||cnn_bar||p.fpga_keys||p.interactive||p.quad_composite||p.nav_composite||p.intersection||p.debug_hud||p.cnn_use_type) {
            fprintf(stderr,"Shortcut mode cannot combine ADAS/CNN/composite/interactive options; unset ADAS_HUD_MODE.\n");return 2;
        }
        if(p.passthrough){p.parallel=0;preprocess="cpu";}
        fprintf(stderr,"RUNTIME_MODE=%s YOLO=%d TINY_CNN=0 ADAS_OVERLAY=0\n",p.passthrough?"passthrough":"yolo_only",p.yolo_only);
    }
    if((p.replay_fps>0&&(!replay||!p.intersection))) bad=1;
    if(bad||(p.fpga_keys&&(!cnn_bar||replay))||(p.cnn_use_type==1&&!cnn_bar&&!p.nav_composite)||(strcmp(preprocess,"cpu")&&strcmp(preprocess,"rga"))) return 2;
    if(p.nav_composite&&p.quad_composite)return 2;
    adas_hud_set_pixel_light(p.model10);
    if(p.nav_composite) {
        adas_hud_set_mode(HUD_MODE_NAV);
        const char *binding=getenv("ADAS_NAV_SIGNAL_ROUTE");
        if(binding&&*binding) {
            if(!strcmp(binding,"left"))p.nav_signal_direction=NAV_LEFT;
            else if(!strcmp(binding,"right"))p.nav_signal_direction=NAV_RIGHT;
            else if(!strcmp(binding,"straight"))p.nav_signal_direction=NAV_STRAIGHT;
            else {fprintf(stderr,"ADAS_NAV_SIGNAL_ROUTE must be left, right or straight\n");return 2;}
        }
        fprintf(stderr,"NAV signal ROI route=%s (installation binding; SEARCH remains candidate)\n",
            adas_nav_direction_name(p.nav_signal_direction));
        fprintf(stderr,"NAV_NATIVE atlas RGB565; road ROI 594,134,1326,870 -> 1920x1080; FPGA image features excluded; CPU adaptive lanes; route advisory only\n");
    }
    if(p.quad_layout&&!p.quad_composite)return 2;
    int front_y=p.quad_layout==QUAD_LAYOUT_REAR_QUARTER?270:QUAD_FRONT_Y0;
    if(p.quad_composite&&((p.debug_hud&&!p.quad_layout)||strcmp(preprocess,"cpu")||
       (p.signal_roi.enabled&&p.signal_roi.y<front_y)))return 2;
    if(p.quad_composite)fprintf(stderr,"四模块单路输入：C 分区预处理，整幅复合帧单次冻结 YOLO，上后视%d..%d，下前视%d..1079；ROI v4 硬件结果可用，自适应模式可回退 CPU。\n",p.quad_layout?0:QUAD_TOP_Y,front_y-1,front_y);
    if(fast_crc)fprintf(stderr,"FAST_CRC：图像 XOR 仅检查首帧及每隔30帧；帧尾结构仍逐帧校验。\n");
    p.cnn.fd=-1;
    struct sigaction action={0}; action.sa_handler=stop_signal; sigemptyset(&action.sa_mask);
    sigaction(SIGINT,&action,NULL); sigaction(SIGTERM,&action,NULL); signal(SIGPIPE,SIG_IGN);
    pthread_mutex_init(&p.mutex,NULL);
    pthread_condattr_t attr; pthread_condattr_init(&attr); pthread_condattr_setclock(&attr,CLOCK_MONOTONIC);
    pthread_cond_init(&p.available,&attr); pthread_condattr_destroy(&attr);
    atomic_init(&p.failed,0);
    int fd=-1,result=1,thread_started=0,infer_started=0,prep_opened=0; FILE *input=NULL; pid_t child=-1; pthread_t worker,infer_worker;
    int stdin_flags=-1,term_changed=0; struct termios original_term;
    void *maps[2]={MAP_FAILED,MAP_FAILED};
    if(cnn_bar) { if(lane_cnn_open(&p.cnn,cnn_bar)) { perror("CNN BAR allocation/ABI (FEC0, trained v2)"); goto cleanup; } p.cnn_enabled=1; }
    if(p.fpga_keys) {
        int ignored;
        if(lane_key_poll(p.cnn.map,&p.keys,&ignored)<0||!p.keys.initialized) {
            fprintf(stderr,"FPGA key ABI missing at BAR FEB0; load the matching key-enabled bitstream.\n");goto cleanup;
        }
        if(p.intersection)fprintf(stderr,"路口按键：M15/Key1单按松开左转，M16/Key2单按松开右转；双键短按松开直行，双键持续1秒停车。按键表示演示指令。\n");
        else fprintf(stderr,"FPGA按键：M15左转、M16右转，同时按下取消；实线红、有效虚线且无占用绿、未知/占用黄。\n");
    }
    if(!p.passthrough&&npu_open(&p.npu,model,library,p.model10)) goto cleanup;
    if(p.passthrough) {
        for(int i=0;i<PREP_SOURCE_SLOTS;i++) {
            p.slots[i].payload=malloc(PAYLOAD_BYTES);
            if(!p.slots[i].payload){perror("capture allocation");goto cleanup;}
        }
    } else {
        if(prep_open(&p.prep,!strcmp(preprocess,"rga"),rga_library,rga_heap)) goto cleanup;
        prep_opened=1;
        for(int i=0;i<PREP_SOURCE_SLOTS;i++) p.slots[i].payload=p.prep.source[i].data;
    }
    if(p.nav_composite)for(int i=0;i<PREP_SOURCE_SLOTS;i++) {
        if(adas_nav_slot_init(&p.slots[i].nav)) { perror("navigation slot allocation"); goto cleanup; }
    }
    if(replay) { input=fopen(replay,"rb"); if(!input) { perror("replay"); goto cleanup; } }
    else {
        fd=open(device,O_RDWR|O_CLOEXEC); struct video_info info={0};
        if(fd<0||ioctl(fd,INFO,&info)<0) { perror("open/INFO"); goto cleanup; }
        if(info.width!=ADAS_W||info.height!=ADAS_H||info.pitch!=ADAS_W*2||info.buffers!=2||info.bytes!=IMAGE_BYTES||sysconf(_SC_PAGESIZE)!=4096) {
            fprintf(stderr,"Unexpected driver geometry/ABI\n"); goto cleanup;
        }
        for(int i=0;i<2;i++) { maps[i]=mmap(NULL,ALLOC_BYTES,PROT_READ,MAP_SHARED,fd,(off_t)i*ALLOC_BYTES); if(maps[i]==MAP_FAILED) { perror("mmap"); goto cleanup; } }
    }
    /* Exclusive output creation avoids overwriting recordings or replay inputs. */
    if(output&&!(p.raw=fopen(output,"wbx"))) { perror("output"); goto cleanup; }
    if(results&&!(p.json=fopen(results,"wx"))) { perror("results"); goto cleanup; }
    if(!headless) { child=start_display(&p.pipe_fd,connector); if(child<0) { perror("display"); goto cleanup; } }
    if(p.interactive) {
        stdin_flags=fcntl(STDIN_FILENO,F_GETFL);
        if(stdin_flags<0||fcntl(STDIN_FILENO,F_SETFL,stdin_flags|O_NONBLOCK)<0) { perror("interactive input"); goto cleanup; }
        if(isatty(STDIN_FILENO)) {
            if(tcgetattr(STDIN_FILENO,&original_term)) goto cleanup;
            struct termios raw=original_term; raw.c_lflag&=~(ICANON|ECHO); raw.c_cc[VMIN]=0; raw.c_cc[VTIME]=0;
            if(tcsetattr(STDIN_FILENO,TCSANOW,&raw)) goto cleanup;
            term_changed=1;
        }
        fprintf(stderr,"模拟按键：A/D 转向，C 取消，Q/E 强制变道，W 行驶，S 停止，R/G/Y/U 灯态，+/- 车速，1/2/3 路线，0 重置，ESC 退出。J/K/L 选择车道，M 切换搜索灯联动；灯态按键覆盖区域识别，U 恢复识别。\n");
        if(p.intersection)fprintf(stderr,"路口覆盖数字键：0停车、1左转、2直行、3右转；其他交互仍限实验室。\n");
    }
    fprintf(stderr,"PIPELINE mode=%s slots=%d EVERY_DISPLAY_FRAME_YOLO=%d YOLO_PASSES_PER_FRAME=%d\n",p.parallel?"parallel":"serial",PREP_SOURCE_SLOTS,!p.passthrough,p.passthrough?0:(1+p.signal_zoom+p.signal_wide));
    int thread_error=pthread_create(&worker,NULL,(p.passthrough||p.yolo_only)?process_shortcut:process_frames,&p);
    if(thread_error) { fprintf(stderr,"CPU worker: %s\n",strerror(thread_error)); goto cleanup; }
    thread_started=1;
    if(p.parallel) {
        thread_error=pthread_create(&infer_worker,NULL,infer_frames,&p);
        if(thread_error) { fprintf(stderr,"NPU worker: %s\n",strerror(thread_error)); atomic_store(&p.failed,1); goto cleanup; }
        infer_started=1;
    }
    if(fd>=0&&!stopping&&start_dma(fd)<0) { if(!stopping) { perror("START"); atomic_store(&p.failed,1); } goto cleanup; }
    struct completion previous={0}; uint32_t last_source=0; int have_previous=0;
    int dma_resynced=0;
    while(!stopping&&!atomic_load(&p.failed)&&(!limit||p.captured<limit)) {
        int index=-1;
        pthread_mutex_lock(&p.mutex);
        for(int i=0;i<PREP_SOURCE_SLOTS;i++) if(!p.slots[i].busy) { index=i; p.slots[i].busy=1; break; }
        pthread_mutex_unlock(&p.mutex);
        if(index<0) { fprintf(stderr,"Internal slot ownership violation\n"); atomic_store(&p.failed,1); break; }
        struct slot *s=&p.slots[index]; double begin=now(); struct completion frame={0};
        if(prep_cpu_begin(&p.prep,index)) { atomic_store(&p.failed,1); break; }
        if(input) {
            size_t n=fread(s->payload,1,PAYLOAD_BYTES,input);
            if(n==0&&!ferror(input)) { pthread_mutex_lock(&p.mutex); s->busy=0; pthread_mutex_unlock(&p.mutex); break; }
            if(n!=PAYLOAD_BYTES) { fprintf(stderr,"Truncated replay frame\n"); atomic_store(&p.failed,1); break; }
            frame=(struct completion){(uint32_t)p.captured,(uint32_t)p.captured+1,(uint32_t)p.captured%2,0};
        } else {
            if(ioctl(fd,WAIT,&frame)<0) { if(!stopping) { perror("WAIT"); atomic_store(&p.failed,1); } break; }
            if(frame.sequence!=((frame.previous+1)&0x3fffffff)||frame.buffer>=2||frame.reserved||
               (have_previous&&!dma_resynced&&(frame.sequence!=((previous.sequence+1)&0x3fffffff)||frame.buffer==previous.buffer))) {
                fprintf(stderr,"DMA completion sequence/buffer violation\n"); atomic_store(&p.failed,1); break;
            }
        }
        s->arrived=now(); s->wait_ms=(s->arrived-begin)*1000;
        if(!input) memcpy(s->payload,maps[frame.buffer],PAYLOAD_BYTES);
        if(adas_parse(s->payload,&s->f,adas_packet_checksum(s->payload,p.captured,fast_crc))) { fprintf(stderr,"Invalid image/FEAT/FVID/LUXF/RFX1 packet\n"); atomic_store(&p.failed,1); break; }
        uint32_t delta=s->f.source_id-last_source;
        if((have_previous&&(!delta||delta>=0x80000000))||(s->f.source_kind&&!allow_test)) {
            fprintf(stderr,"Repeated/reversed source frame or internal test source (requires --allow-internal-test)\n"); atomic_store(&p.failed,1); break;
        }
        s->f.source_kind|=ADAS_SOURCE_STABLE_HUD;
        if(p.quad_layout==QUAD_LAYOUT_REAR_QUARTER)s->f.source_kind|=ADAS_SOURCE_QUAD_QUARTER;
        if(p.quad_composite)adas_quad_features(s->payload,&s->f);
        if(p.nav_composite&&adas_nav_capture(&s->nav,s->payload,&s->f)) {
            fprintf(stderr,"Navigation capture/road isolation failed\n");atomic_store(&p.failed,1);break;
        }
        if(prep_cpu_end(&p.prep,index)) { atomic_store(&p.failed,1); break; }
        /* Finish the CPU snapshot before granting the next DMA buffer.
         * NPU inference and display still run on their separate workers. */
        dma_resynced=0;
        if(fd>=0&&!stopping&&(!limit||p.captured+1<limit)) {
            int started=start_dma(fd);
            if(started<0) { if(!stopping) { perror("next START"); atomic_store(&p.failed,1); } break; }
            /* STOP can consume a cancelled/completed grant. The driver still
             * validates every marker; rebase only our cross-frame comparison. */
            dma_resynced=started;
            if(started) fprintf(stderr,"DMA_RECOVERY resumed; completion sequence rebased\n");
        }
        s->sequence=frame.sequence; s->copy_ms=(now()-s->arrived)*1000; s->enqueued=now();
        previous=frame; last_source=s->f.source_id; have_previous=1;
        pthread_mutex_lock(&p.mutex);
        p.captured++;
        if(p.pending>=0) { p.slots[p.pending].busy=0; p.dropped++; }
        p.pending=index; pthread_cond_broadcast(&p.available);
        pthread_mutex_unlock(&p.mutex);
    }
    result=atomic_load(&p.failed)?1:0;
cleanup:
    /* STOP drains the outstanding grant before any DMA mapping is released. */
    if(fd>=0&&ioctl(fd,STOP)<0) { perror("STOP/drain"); result=1; }
    pthread_mutex_lock(&p.mutex); p.done=1; if(!infer_started) p.infer_done=1;
    pthread_cond_broadcast(&p.available); pthread_mutex_unlock(&p.mutex);
    if(infer_started) pthread_join(infer_worker,NULL);
    if(thread_started) pthread_join(worker,NULL);
    if(term_changed) tcsetattr(STDIN_FILENO,TCSANOW,&original_term);
    if(stdin_flags>=0) fcntl(STDIN_FILENO,F_SETFL,stdin_flags);
    if(atomic_load(&p.failed)) result=1;
    if(p.pipe_fd>=0) close(p.pipe_fd);
    if(finish_display(child)&&!stopping) result=1;
    if(input) fclose(input);
    if(p.raw&&fclose(p.raw)) result=1;
    if(p.json&&fclose(p.json)) result=1;
    if(p.cnn_enabled)lane_cnn_close(&p.cnn);
    npu_close(&p.npu);
    for(int i=0;i<2;i++) if(maps[i]!=MAP_FAILED) munmap(maps[i],ALLOC_BYTES);
    if(fd>=0) close(fd);
    for(int i=0;i<PREP_SOURCE_SLOTS;i++)adas_nav_slot_close(&p.slots[i].nav);
    if(prep_opened) prep_close(&p.prep);
    if(p.passthrough)for(int i=0;i<PREP_SOURCE_SLOTS;i++)free(p.slots[i].payload);
    pthread_cond_destroy(&p.available); pthread_mutex_destroy(&p.mutex);
    printf("SUMMARY captured=%"PRIu64" processed=%"PRIu64" dropped=%"PRIu64" result=%d\n",p.captured,p.processed,p.dropped,result);
    return result;
}
