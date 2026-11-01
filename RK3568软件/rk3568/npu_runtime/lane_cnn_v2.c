#define _GNU_SOURCE
#include "lane_cnn_v2.h"
#include "lane_roi_config.h"
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

int lane_cnn_decode(const uint32_t w[12],struct lane_cnn_snapshot *s) {
    if(w[0]!=UINT32_C(0x314e414c)||(w[1]&65535)!=1)return -1;
    if((w[2]&1)||w[2]!=w[10])return 0;
    struct lane_cnn_snapshot value={.sequence=w[2],.flags=w[3],.frame_id=w[4],
        .roi_version=w[7],.drops=w[11],.model_version=w[1]>>16};
    for(unsigned i=0;i<2;i++) {
        value.cls[i]=(w[5]>>(i*2))&3;
        value.valid[i]=(w[5]>>(4+i))&1;
        value.confidence[i]=(w[6]>>(i*16))&65535;
        value.margin[i]=w[8+i];
        if(value.valid[i]!=(value.cls[i]!=3))return -1;
    }
    *s=value;return 1;
}

int lane_cnn_read_words(const volatile uint32_t *map,struct lane_cnn_snapshot *s) {
    for(unsigned attempt=0;attempt<50;attempt++) {
        uint32_t first=map[(LANE_CNN_BAR_BASE+8)/4];
        if(first&1)continue;
        __sync_synchronize();uint32_t words[12];
        for(unsigned i=0;i<12;i++)words[i]=map[LANE_CNN_BAR_BASE/4+i];
        __sync_synchronize();uint32_t last=map[(LANE_CNN_BAR_BASE+8)/4];
        if(first!=last||first!=words[2]||first!=words[10])continue;
        int decoded=lane_cnn_decode(words,s);if(decoded)return decoded;
    }
    return 0;
}

int lane_cnn_open(struct lane_cnn_reader *r,const char *resource0) {
    memset(r,0,sizeof(*r));r->fd=-1;
    char parent[4096],path[4096];
    if(strlen(resource0)>=sizeof(parent)-16){errno=ENAMETOOLONG;return -1;}
    strcpy(parent,resource0);char *suffix=strrchr(parent,'/');
    if(!suffix||strcmp(suffix,"/resource0")){errno=EINVAL;return -1;}*suffix=0;
    snprintf(path,sizeof(path),"%s/config",parent);int config=open(path,O_RDONLY|O_CLOEXEC);
    uint8_t bytes[24];ssize_t n=config<0?-1:pread(config,bytes,sizeof(bytes),0);
    if(config>=0)close(config);
    snprintf(path,sizeof(path),"%s/resource",parent);FILE *resource=fopen(path,"r");
    unsigned long long start=0,end=0,flags=0;
    int fields=resource?fscanf(resource,"%llx %llx %llx",&start,&end,&flags):0;
    if(resource)fclose(resource);
    uint32_t base=n==24?(uint32_t)bytes[16]|((uint32_t)bytes[17]<<8)|((uint32_t)bytes[18]<<16)|((uint32_t)bytes[19]<<24):0;
    if(n!=24||fields!=3||!start||end<start||end-start!=65535||!(flags&0x200)||
       (base&15)||(base&UINT32_C(0xfffffff0))!=start||(bytes[4]&6)!=6){errno=ENODEV;return -1;}
    r->fd=open(resource0,O_RDONLY|O_CLOEXEC);if(r->fd<0)return -1;
    void *map=mmap(NULL,65536,PROT_READ,MAP_SHARED,r->fd,0);
    if(map==MAP_FAILED){close(r->fd);r->fd=-1;return -1;}
    r->map=map;
    if(lane_cnn_read_words(r->map,&r->snapshot)!=1){lane_cnn_close(r);errno=ENODEV;return -1;}
    r->observed=1;
    return 0;
}

void lane_cnn_close(struct lane_cnn_reader *r) {
    if(r->map)munmap((void *)r->map,65536);
    if(r->fd>=0)close(r->fd);
    r->map=NULL;r->fd=-1;
}

int lane_cnn_poll(struct lane_cnn_reader *r,double now) {
    struct lane_cnn_snapshot next;int result=lane_cnn_read_words(r->map,&next);
    if(result!=1){r->progress=0;return result;}
    if(!r->observed||next.sequence!=r->snapshot.sequence) {
        uint32_t delta=next.sequence-r->snapshot.sequence;
        r->progress=r->observed&&delta>0&&delta<UINT32_C(0x80000000);
        r->changed=now;r->snapshot=next;r->observed=1;
        if(!r->progress){r->history_count=0;r->history_next=0;}
        else {
            unsigned slot=r->history_next;r->history[slot]=next;r->history_changed[slot]=now;
            r->history_next=(slot+1)%4;if(r->history_count<4)r->history_count++;
        }
    }
    return 1;
}

const struct lane_cnn_snapshot *lane_cnn_for_frame(const struct lane_cnn_reader *r,uint32_t source_id,double now) {
    if(!r->observed||!r->progress)return NULL;
    const struct lane_cnn_snapshot *best=NULL;uint32_t best_age=9;
    unsigned count=r->history_count?r->history_count:1;
    for(unsigned i=0;i<count;i++) {
        const struct lane_cnn_snapshot *s=r->history_count?&r->history[i]:&r->snapshot;
        double changed=r->history_count?r->history_changed[i]:r->changed;
        uint32_t age=source_id-s->frame_id;
        if(s->model_version==2&&(s->roi_version==3||s->roi_version==4)&&(s->flags&1)&&!(s->flags&4)&&
           age<best_age&&now>=changed&&now-changed<=.25){best=s;best_age=age;}
    }
    return best;
}

int lane_cnn_fresh(const struct lane_cnn_reader *r,uint32_t source_id,double now) {
    return lane_cnn_for_frame(r,source_id,now)!=NULL;
}

unsigned lane_cnn_apply(const struct lane_cnn_reader *r,uint32_t source_id,double now,struct lane lanes[2]) {
    const struct lane_cnn_snapshot *s=lane_cnn_for_frame(r,source_id,now);
    if(!s)return 0;
    unsigned applied=0;
    for(unsigned i=0;i<2;i++)if(lanes[i].valid&&s->valid[i]&&s->cls[i]<3) {
        if(!lane_roi_aligned_version(&lanes[i],(int)i,s->roi_version))continue;
        unsigned cls=s->cls[i];
        lanes[i].type=adas_lane_type((enum lane_class)cls);
        applied|=1u<<i;
    }
    return applied;
}

const char *lane_cnn_label(unsigned cls) {
    static const char *const labels[]={"NONE","DASHED","SOLID","INVALID"};
    return labels[cls<4?cls:3];
}
void lane_cnn_display_update(struct lane_cnn_display *d,
    const struct lane_cnn_snapshot *s,unsigned side,double now,int reset) {
    if(reset||!d->initialized) { memset(d,0,sizeof(*d)); d->cls=LANE_CLASS_UNKNOWN; d->initialized=1; }
    if(side>1) { d->cls=LANE_CLASS_INVALID; d->held=0; return; }
    if(!s) {
        d->count=0;
        if(d->cls<LANE_CLASS_INVALID&&d->missing<12&&now>=d->last_valid&&now-d->last_valid<=.25) {
            d->missing++; d->held=1;
        } else { d->cls=LANE_CLASS_UNKNOWN; d->held=0; }
        return;
    }
    if(!s->valid[side]||s->cls[side]>LANE_CLASS_SOLID) {
        d->cls=LANE_CLASS_INVALID; d->count=0; d->held=0; d->have_sample=0; return;
    }
    d->missing=0; d->held=0;
    /* Count distinct CNN results rather than repeated video frames. */
    if(d->have_sample&&d->sequence==s->sequence) return;
    d->have_sample=1; d->sequence=s->sequence; d->frame=s->frame_id; d->last_valid=now;
    if(d->count<3) d->hist[d->count++]=s->cls[side];
    else { d->hist[0]=d->hist[1]; d->hist[1]=d->hist[2]; d->hist[2]=s->cls[side]; }
    unsigned votes[3]={0};
    for(unsigned i=0;i<d->count;i++) votes[d->hist[i]]++;
    d->cls=LANE_CLASS_UNKNOWN;
    for(unsigned i=0;i<3;i++) if(votes[i]>=2||d->count==1) {
        if(votes[i]) { d->cls=(enum lane_class)i; break; }
    }
}

int lane_key_poll(const volatile uint32_t *map,struct lane_key_reader *r,int *turn) {
    if(!map||!r||!turn)return -1;
    for(unsigned attempt=0;attempt<50;attempt++) {
        const volatile uint32_t *p=map+LANE_KEY_BAR_BASE/4;
        uint32_t sequence=p[3];if(sequence&1)continue;
        __sync_synchronize();uint32_t magic=p[0],left=p[1],right=p[2];
        __sync_synchronize();if(sequence!=p[3])continue;
        if((magic&UINT32_C(0x3fffffff))!=UINT32_C(0x3159454b))return -1;
        uint32_t dl=left-r->left,dr=right-r->right,delta=sequence-r->sequence;
        int have=r->initialized;
        int baseline=!have||delta>=UINT32_C(0x80000000)||
            dl>=UINT32_C(0x80000000)||dr>=UINT32_C(0x80000000)||(!delta&&(dl||dr));
        r->initialized=1;r->sequence=sequence;r->left=left;r->right=right;
        if(baseline) { /* Reconfiguration clears a request; startup ignores historical presses. */
            if(have){*turn=0;return 1;}
            return 0;
        }
        if(!dl&&!dr)return 0;
        *turn=(magic>>30)==3||((dl!=0)==(dr!=0))?0:dl?-1:1;
        return 1;
    }
    return 0;
}
