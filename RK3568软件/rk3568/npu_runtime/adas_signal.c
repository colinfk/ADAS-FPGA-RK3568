#include "adas_signal.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include "light_params_v2.h"
#define LIGHT_ACQUIRE_FRAMES 2
#define LIGHT_SWITCH_FRAMES 5
#define LIGHT_SWITCH_DWELL 12
#define LIGHT_SWITCH_RATIO 1.15f
#define LIGHT_HOLD_FRAMES 24
#define LIGHT_MATCH_IOU .15f
#define LIGHT_MATCH_DIST 1.25f
#define LIGHT_REACQ_MIN 60
#define LIGHT_CHALLENGE_MIN 60
#define LIGHT_CHALLENGE_RATIO 16
#define LIGHT_CHALLENGE_FRAMES 3
void adas_light_track_reset(struct light_track *t) { memset(t,0,sizeof(*t)); t->selected=-1; }
static float light_distance(const struct light_track *t,const struct detection *b) {
    return hypotf((t->x0+t->x1-b->x0-b->x1)*.5f,(t->y0+t->y1-b->y0-b->y1)*.5f);
}
static int light_match(const struct light_track *t,const struct detection *b) {
    float w=fmaxf(0,fminf(t->x1,b->x1)-fmaxf(t->x0,b->x0));
    float h=fmaxf(0,fminf(t->y1,b->y1)-fmaxf(t->y0,b->y0));
    float area=w*h,denom=(t->x1-t->x0)*(t->y1-t->y0)+(b->x1-b->x0)*(b->y1-b->y0)-area;
    return (denom>0&&area/denom>=LIGHT_MATCH_IOU)||light_distance(t,b)<=LIGHT_MATCH_DIST*fmaxf(b->x1-b->x0,b->y1-b->y0);
}
static void light_copy_box(struct light_track *t,const struct detection *b) {
    t->x0=b->x0;t->y0=b->y0;t->x1=b->x1;t->y1=b->y1;
}
static void light_show(const struct light_track *t,enum adas_light *color,struct candidate *box) {
    *color=t->color;
    *box=(struct candidate){1,(unsigned)t->x0,(unsigned)t->y0,(unsigned)t->x1,(unsigned)t->y1,
        (unsigned)((t->x0+t->x1)*.5f),(unsigned)((t->y0+t->y1)*.5f),0};
}
int adas_light_track_step(struct light_track *t,const struct detection *boxes,const enum adas_light *colors,
    const int *red,const int *green,const float *scores,int count,int fallback,int reset,uint32_t fid,
    enum adas_light *shown_color,struct candidate *shown_box) {
    (void)scores;
    if(reset)adas_light_track_reset(t);
    t->last_fid=fid;*shown_color=LIGHT_UNKNOWN;memset(shown_box,0,sizeof(*shown_box));
    int m=-1,geom=-1;
    if(t->valid)for(int i=0;i<count;i++)if(light_match(t,&boxes[i])) {
        if(colors[i]==LIGHT_UNKNOWN) { if(geom<0||light_distance(t,&boxes[i])<light_distance(t,&boxes[geom]))geom=i;continue; }
        if(m<0||(colors[i]==t->color&&colors[m]!=t->color)||
           ((colors[i]==t->color)==(colors[m]==t->color)&&light_distance(t,&boxes[i])<light_distance(t,&boxes[m])))m=i;
    }
    int src=t->valid?m:fallback;
    int chal=-1,chal_v=0;
    if(t->allow_switch)for(int i=0;i<count;i++)if(colors[i]!=LIGHT_UNKNOWN) {
        int v=red[i]+green[i];if(v>chal_v){chal_v=v;chal=i;}
    }
    int snap=0;
    if(t->allow_switch&&chal>=0) {
        if(src<0&&t->valid&&chal_v>=LIGHT_REACQ_MIN)snap=1;
        else if(src>=0&&src<count&&chal!=src) {
            int cur=red[src]+green[src];
            int ok=chal_v>=LIGHT_CHALLENGE_MIN&&chal_v*10>=cur*LIGHT_CHALLENGE_RATIO;
            t->challenge_run=ok?t->challenge_run+1:0;
            if(t->challenge_run>=LIGHT_CHALLENGE_FRAMES)snap=1;
        } else t->challenge_run=0;
        if(snap) {
            t->valid=1;t->acquired=1;t->color=colors[chal];
            light_copy_box(t,&boxes[chal]);
            t->dwell=0;t->pending_run=0;t->pending=colors[chal];
            t->hold=LIGHT_HOLD_FRAMES;t->challenge_run=0;t->selected=chal;src=chal;
        }
    } else if(t->allow_switch)t->challenge_run=0;
    if(src<0||src>=count||colors[src]==LIGHT_UNKNOWN) {
        if(!t->valid) { t->pending_run=0;return 0; }
        if(geom>=0) {
            t->x0=.7f*t->x0+.3f*boxes[geom].x0;t->y0=.7f*t->y0+.3f*boxes[geom].y0;
            t->x1=.7f*t->x1+.3f*boxes[geom].x1;t->y1=.7f*t->y1+.3f*boxes[geom].y1;
        }
        t->pending_run=0;
        if(t->hold>0) { t->hold--;t->dwell++;light_show(t,shown_color,shown_box);return 1; }
        t->valid=0;t->selected=-1;t->pending_run=0;return 0;
    }
    enum adas_light raw=colors[src];
    if(!t->valid) {
        if(raw==t->pending)t->pending_run++;else {t->pending=raw;t->pending_run=1;}
        if(t->pending_run<(t->acquired?LIGHT_ACQUIRE_FRAMES:1))return 0;
        t->valid=1;t->acquired=1;t->color=raw;t->dwell=0;t->hold=LIGHT_HOLD_FRAMES;
        light_copy_box(t,&boxes[src]);
    } else {
        t->hold=LIGHT_HOLD_FRAMES;t->dwell++;
        int current=t->color==LIGHT_RED?red[src]:t->color==LIGHT_GREEN?green[src]:0;
        int advantage=red[src]+green[src]>=current*LIGHT_SWITCH_RATIO;
        if(raw==t->color)t->pending_run=0;
        else {
            if(raw==t->pending&&advantage)t->pending_run++;
            else {t->pending=raw;t->pending_run=advantage?1u:0u;}
            if(t->pending_run>=LIGHT_SWITCH_FRAMES&&t->dwell>=LIGHT_SWITCH_DWELL) {
                t->color=raw;t->dwell=0;t->pending_run=0;
            }
        }
    }
    t->x0=.7f*t->x0+.3f*boxes[src].x0;t->y0=.7f*t->y0+.3f*boxes[src].y0;
    t->x1=.7f*t->x1+.3f*boxes[src].x1;t->y1=.7f*t->y1+.3f*boxes[src].y1;
    t->selected=src;light_show(t,shown_color,shown_box);return 1;
}
static void light_conv(const int8_t *a,int8_t *b,int h,int w,int ci,int co,
                       const int8_t *weights,const int32_t *bias,unsigned shift) {
    for(int y=0;y<h;y++)for(int x=0;x<w;x++)for(int oc=0;oc<co;oc++) {
        int64_t sum=bias[oc];
        for(int ic=0;ic<ci;ic++)for(int ky=0;ky<3;ky++)for(int kx=0;kx<3;kx++) {
            int yy=y+ky-1,xx=x+kx-1;
            if(yy>=0&&yy<h&&xx>=0&&xx<w)
                sum+=(int)a[(yy*w+xx)*ci+ic]*(int)weights[((oc*ci+ic)*3+ky)*3+kx];
        }
        sum=sum<0?0:(sum+(INT64_C(1)<<(shift-1)))>>shift;
        b[(y*w+x)*co+oc]=(int8_t)(sum>127?127:sum);
    }
}
void light_color_max_infer(const uint8_t pixels[32*32*3],int32_t logits[4]) {
    int8_t input[32*32*3],first[32*32*4],pooled[16*16*4],second[16*16*8],gap[8]={0};
    for(int i=0;i<32*32*3;i++)input[i]=(int8_t)(pixels[i]>>1);
    light_conv(input,first,32,32,3,4,light_conv1_w,light_conv1_b,9);
    for(int y=0;y<16;y++)for(int x=0;x<16;x++)for(int c=0;c<4;c++) {
        int8_t value=0;
        for(int dy=0;dy<2;dy++)for(int dx=0;dx<2;dx++) {
            int8_t v=first[((y*2+dy)*32+x*2+dx)*4+c];if(v>value)value=v;
        }
        pooled[(y*16+x)*4+c]=value;
    }
    light_conv(pooled,second,16,16,4,8,light_conv2_w,light_conv2_b,8);
    for(int p=0;p<16*16;p++)for(int c=0;c<8;c++)if(second[p*8+c]>gap[c])gap[c]=second[p*8+c];
    for(int out=0;out<4;out++) {
        int32_t sum=light_fc_b[out];
        for(int c=0;c<8;c++)sum+=(int)gap[c]*(int)light_fc_w[out*8+c];
        logits[out]=sum;
    }
}
int light_color_max_input(const uint8_t *image,int x0,int y0,int x1,int y1,uint8_t pixels[32*32*3]) {
    if(!image||!pixels||x0<0||y0<0||x1>ADAS_W||y1>ADAS_H||x1<=x0||y1<=y0)return 0;
    int w=x1-x0,h=y1-y0,nw=32,nh=32;
    if(h>=w)nw=w*32/h;else nh=h*32/w;
    if(nw<1)nw=1;
    if(nh<1)nh=1;
    memset(pixels,0,32*32*3);
    for(int y=0;y<nh;y++)for(int x=0;x<nw;x++) {
        const uint8_t *p=image+((y0+y*h/nh)*ADAS_W+x0+x*w/nw)*2;
        unsigned word=p[0]|(unsigned)p[1]<<8,r=word>>11,g=(word>>5)&63,b=word&31;
        uint8_t *q=pixels+(((32-nh)/2+y)*32+(32-nw)/2+x)*3;
        q[0]=(uint8_t)((r<<3)|(r>>2));q[1]=(uint8_t)((g<<2)|(g>>4));q[2]=(uint8_t)((b<<3)|(b>>2));
    }
    return 1;
}
#define GRID_PIXELS (960*540)
int adas_signal_init(struct signal_detector *s) {
    memset(s,0,sizeof(*s)); s->mask=malloc(GRID_PIXELS); s->queue=malloc(GRID_PIXELS*sizeof(int));
    if(!s->mask||!s->queue) { adas_signal_close(s); return -1; }
    return 0;
}
void adas_signal_close(struct signal_detector *s) { free(s->mask); free(s->queue); memset(s,0,sizeof(*s)); }
static void channels(const uint8_t *image,int x,int y,int *r,int *g,int *b) {
    const uint8_t *p=image+(y*ADAS_W+x)*2; unsigned v=p[0]|(unsigned)p[1]<<8;
    *r=(v>>11)<<3; *g=((v>>5)&63)<<2; *b=(v&31)<<3;
}
#define LIGHT_MIN_MAX 150
#define LIGHT_RED_D 20
#define LIGHT_RED_B 20
#define LIGHT_GREEN_R 12
#define LIGHT_GREEN_B 8
#define LIGHT_MIN_VOTES 20
#define LIGHT_LIT_FRAC 10
#define LIGHT_RING_DIFF -5
enum adas_light pixel_light_color(const uint8_t *image,const struct detection *box,int *red_out,int *green_out) {
    if(red_out)*red_out=0;
    if(green_out)*green_out=0;
    if(!image||!box||!isfinite(box->x0)||!isfinite(box->y0)||!isfinite(box->x1)||!isfinite(box->y1))return LIGHT_UNKNOWN;
    int x0=(int)fmaxf(0,box->x0),y0=(int)fmaxf(0,box->y0);
    int x1=(int)fminf(ADAS_W-1,box->x1),y1=(int)fminf(ADAS_H-1,box->y1);
    int w=x1-x0+1,h=y1-y0+1;
    if(x0>=ADAS_W||y0>=ADAS_H||w<6||h<8||w>192||h>237)return LIGHT_UNKNOWN;
    int red=0,green=0,yellow=0,green_core=0,lit=0,n=0,sum=0,step=(w>48||h>48)?2:1;
    for(int y=y0;y<=y1;y+=step)for(int x=x0;x<=x1;x+=step) {
        int r,g,b;channels(image,x,y,&r,&g,&b);
        int m=r>g?(r>b?r:b):(g>b?g:b); sum+=m;n++;lit+=m>=160;
        int low=r<g?(r<b?r:b):(g<b?g:b);
        if(m<160||m-low<16||(m-low)*100<m*15)continue;
        if(r>=150&&g>=140&&abs(r-g)<=80&&b+60<(r<g?r:g))yellow++;
        else if(r>=LIGHT_MIN_MAX&&r>=g+LIGHT_RED_D&&r>=b+LIGHT_RED_B&&(step==1||g*100<r*65))red++;
        else if(g>=120&&g>=r+LIGHT_GREEN_R&&(g>=b+LIGHT_GREEN_B||(g>=180&&g+12>=b))) {green++;if(g>=(step==2?232:240))green_core++;}
    }
    if(red_out)*red_out=red;
    if(green_out)*green_out=green;
    int margin=h/2>3?h/2:3,ring=0,rn=0;
    for(int y=y0-margin;y<=y1+margin;y+=2)for(int x=x0-margin;x<=x1+margin;x+=2) {
        if(x<0||x>=ADAS_W||y<0||y>=ADAS_H||(x>=x0&&x<=x1&&y>=y0&&y<=y1))continue;
        int r,g,b;channels(image,x,y,&r,&g,&b);ring+=(r+2*g+b)/4;rn++;
    }
    if(!rn||lit*100<n*LIGHT_LIT_FRAC)return LIGHT_UNKNOWN;
    if(yellow>=LIGHT_MIN_VOTES&&yellow*2>=red*3&&yellow*2>=green*3)return LIGHT_YELLOW;
    if((sum/n<ring/rn+LIGHT_RING_DIFF&&red+green<20)||red+green<LIGHT_MIN_VOTES)return LIGHT_UNKNOWN;
    if(red>=LIGHT_MIN_VOTES && red*2>=green*3)return LIGHT_RED;
    /* shortcut: dim green lamps may be unknown; replace with calibrated lamp classifier. */
    if(green>=LIGHT_MIN_VOTES && green_core>=3 && green*2>=red*3)return LIGHT_GREEN;
    return LIGHT_UNKNOWN;
}
static unsigned color(int r,int g,int b,int allow_red,int allow_green) {
    if(allow_red&&r>=160&&r>=g+48&&r>=b+48) return LIGHT_RED;
    if(allow_green==2&&g>=150&&g>=r+40&&g+5>=b) return LIGHT_GREEN;
    if(allow_green==1&&g>=160&&g>=r+48&&g>=b+48) return LIGHT_GREEN;
    if(r>=160&&g>=140&&abs(r-g)<=80&&b+60<(r<g?r:g)) return LIGHT_YELLOW;
    return LIGHT_UNKNOWN;
}
static float dark_surround(const uint8_t *image,int x0,int y0,int x1,int y1) {
    int margin=(x1-x0)/2+4,total=0,dark=0;
    for(int y=y0-margin;y<=y1+margin;y+=2) for(int x=x0-margin;x<=x1+margin;x+=2) {
        if(x<0||x>=ADAS_W||y<0||y>=ADAS_H||(x>=x0&&x<=x1&&y>=y0&&y<=y1)) continue;
        int r,g,b; channels(image,x,y,&r,&g,&b); total++; dark+=r<110&&g<110&&b<110;
    }
    return total?(float)dark/total:0;
}
static int gray(const uint8_t *image,int x,int y) {
    int r,g,b; channels(image,x,y,&r,&g,&b); return (r+2*g+b)/4;
}
static int housing(const uint8_t *image,int x0,int y0,int x1,int y1,unsigned type) {
    int diameter=x1-x0+1;
    if(y1-y0+1>diameter) diameter=y1-y0+1;
    int cx=(x0+x1)/2,cy=(y0+y1)/2,pitch=diameter*3/2;
    int slot=type==LIGHT_RED?0:type==LIGHT_YELLOW?1:2;
    /* A round color blob alone is also a tail lamp or sign. Require a dark,
     * elongated three-lamp housing with contrast against its outer sides. */
    for(int horizontal=0;horizontal<2;horizontal++) {
        int first_x=cx-(horizontal?slot*pitch:0),first_y=cy-(horizontal?0:slot*pitch);
        int lo_x=first_x-diameter/2,lo_y=first_y-diameter/2;
        int hi_x=first_x+(horizontal?2*pitch:0)+diameter/2;
        int hi_y=first_y+(horizontal?0:2*pitch)+diameter/2;
        int margin=diameter/2+3;
        if(lo_x-margin<0||lo_y-margin<0||hi_x+margin>=ADAS_W||hi_y+margin>=ADAS_H) continue;
        int total=0,dark=0,sum=0,outer=0,outer_count=0;
        for(int y=lo_y;y<=hi_y;y+=2) for(int x=lo_x;x<=hi_x;x+=2) {
            if(x>=x0&&x<=x1&&y>=y0&&y<=y1) continue;
            int v=gray(image,x,y); total++; sum+=v; dark+=v<105;
        }
        if(horizontal) for(int x=lo_x;x<=hi_x;x+=2) {
            outer+=gray(image,x,lo_y-margin)+gray(image,x,hi_y+margin); outer_count+=2;
        } else for(int y=lo_y;y<=hi_y;y+=2) {
            outer+=gray(image,lo_x-margin,y)+gray(image,hi_x+margin,y); outer_count+=2;
        }
        if(total&&outer_count&&dark*100>=total*65&&outer/outer_count>=sum/total+14) return 1;
    }
    return 0;
}
static unsigned quad_video_color(int r,int g,int b) {
    if(r>=140&&r>=g+40&&r>=b+32)return LIGHT_RED;
    if(g>=130&&g>=r+32&&g>=b+16)return LIGHT_GREEN;
    if(r>=150&&g>=130&&abs(r-g)<=70&&b+40<(r<g?r:g))return LIGHT_YELLOW;
    return LIGHT_UNKNOWN;
}
static float quad_video_housing(const uint8_t *image,int cx,int cy,int diameter,unsigned type,struct candidate *box) {
    float best=0;int radius=diameter/4;if(radius<1)radius=1;
    int slot=type==LIGHT_RED?0:type==LIGHT_YELLOW?1:2;
    const int horizontal=1,direction=-1; /* This video's Japanese horizontal lamp: green-left/red-right. */
    {
        for(int scale=4;scale<=6;scale++) {
            int pitch=diameter*scale/4,dx=horizontal?direction*pitch:0,dy=horizontal?0:pitch;
            int first_x=cx-slot*dx,first_y=cy-slot*dy;
            int x0=(dx<0?first_x+2*dx:first_x)-diameter/2-2;
            int x1=(dx>0?first_x+2*dx:first_x)+diameter/2+3;
            int y0=first_y-diameter/2-2,y1=first_y+2*dy+diameter/2+3;
            if(x0<1||y0<1||x1>=ADAS_W-1||y1>=ADAS_H-1||x1-x0>128||y1-y0>128)continue;
            float contrast=255;int valid=1;
            for(int lamp=0;lamp<3;lamp++)if(lamp!=slot) {
                int x=first_x+lamp*dx,y=first_y+lamp*dy;
                float inside=(gray(image,x,y)+gray(image,x-radius,y)+gray(image,x+radius,y)+
                              gray(image,x,y-radius)+gray(image,x,y+radius))/5.f;
                int margin=diameter/2+3;
                float outer=horizontal?(gray(image,x,y-margin)+gray(image,x,y+margin))*.5f:
                    (gray(image,x-margin,y)+gray(image,x+margin,y))*.5f;
                if(inside>110||outer-inside<24){valid=0;break;}
                contrast=fminf(contrast,outer-inside);
            }
            if(valid&&contrast>best) {
                best=contrast;*box=(struct candidate){1,(unsigned)x0,(unsigned)y0,(unsigned)x1,(unsigned)y1,
                    (unsigned)cx,(unsigned)cy,0};
            }
        }
    }
    return best;
}
static void quad_video_signal(struct signal_detector *s,const uint8_t *image,int x0,int y0,int w,int h,
                         const struct detection *objects,int count,double arrived,struct adas_signal *best,float *rival,int allow_pixel) {
    struct adas_signal candidates[4];int found=0;
    for(int y=0;y<h;y++)for(int x=0;x<w;x++) {
        int r,g,b;channels(image,x0+x*2,y0+y*2,&r,&g,&b);
        s->mask[y*w+x]=(uint8_t)quad_video_color(r,g,b);
    }
    for(int i=0;i<w*h;i++)if(s->mask[i]) {
        unsigned type=s->mask[i];int begin=0,end=1,minx=i%w,maxx=minx,miny=i/w,maxy=miny;
        s->queue[0]=i;s->mask[i]=0;
        while(begin<end) {
            int v=s->queue[begin++],x=v%w,y=v/w;
            if(x<minx)minx=x;
            if(x>maxx)maxx=x;
            if(y<miny)miny=y;
            if(y>maxy)maxy=y;
            for(int dy=-1;dy<=1;dy++)for(int dx=-1;dx<=1;dx++) {
                int xx=x+dx,yy=y+dy;
                if(xx<0||xx>=w||yy<0||yy>=h)continue;
                int at=yy*w+xx;
                if(s->mask[at]==type){s->mask[at]=0;s->queue[end++]=at;}
            }
        }
        int bw=maxx-minx+1,bh=maxy-miny+1;float ratio=(float)bw/bh,fill=(float)end/(bw*bh);
        if(end<3||bw<2||bh<2||bw>16||bh>16||ratio<.55f||ratio>1.8f||fill<.4f||fill>.96f)continue;
        int cx=x0+minx+maxx,cy=y0+miny+maxy,blocked=0;
        for(int k=0;k<count;k++)if((allow_pixel?(objects[k].cls>=0&&objects[k].cls<3):(objects[k].cls<5))&&
            cx>=objects[k].x0&&cx<=objects[k].x1&&
            cy>=objects[k].y0-(allow_pixel?(objects[k].y1-objects[k].y0)*.15f:0.f)&&cy<=objects[k].y1)blocked=1;
        if(blocked)continue;
        struct candidate box={0};int diameter=(bw>bh?bw:bh)*2,step=diameter/6;
        if(step<1)step=1;
        int core=0;
        for(int k=0;k<5;k++) {
            int dx=k==1?-step:k==2?step:0,dy=k==3?-step:k==4?step:0;
            int r,g,b;channels(image,cx+dx,cy+dy,&r,&g,&b);core+=quad_video_color(r,g,b)==type;
        }
        if(core<3)continue; /* Lit cores are filled; a red sign ring is not a lamp. */
        float contrast=quad_video_housing(image,cx,cy,diameter,type,&box);
        float score=.65f+.15f*fminf(1,contrast/64)+.10f*fill;
        if(!box.valid&&arrived>s->last_time&&arrived-s->last_time<.5&&
           s->pending.box.valid&&s->pending.candidate==(enum adas_light)type&&
           abs(cx-(int)s->pending.box.cx)<=24&&abs(cy-(int)s->pending.box.cy)<=24) {
            /* Track a verified housing, but still require current color pixels and INT8 agreement. */
            struct candidate previous=s->pending.box;
            int dx=cx-(int)previous.cx,dy=cy-(int)previous.cy;
            int bx0=(int)previous.x0+dx,by0=(int)previous.y0+dy,bx1=(int)previous.x1+dx,by1=(int)previous.y1+dy;
            if(bx0>=0&&by0>=0&&bx1<ADAS_W&&by1<ADAS_H) {
                box=(struct candidate){1,(unsigned)bx0,(unsigned)by0,(unsigned)bx1,(unsigned)by1,
                    (unsigned)cx,(unsigned)cy,0};score=.72f+.18f*fill;
            }
        }
        if(!box.valid)continue;
        struct adas_signal candidate={(enum adas_light)type,best->source,box,score,0};
        if(found<4)candidates[found++]=candidate;
        else {
            int weakest=0;for(int k=1;k<4;k++)if(candidates[k].confidence<candidates[weakest].confidence)weakest=k;
            if(score>candidates[weakest].confidence)candidates[weakest]=candidate;
        }
    }
    for(int k=0;k<found;k++) {
        struct adas_signal *candidate=&candidates[k];uint8_t pixels[3072];int32_t logits[4];
        struct candidate box=candidate->box;
        if(!light_color_max_input(image,(int)box.x0,(int)box.y0,(int)box.x1,(int)box.y1,pixels))continue;
        light_color_max_infer(pixels,logits);int cls=0;
        for(int j=1;j<4;j++)if(logits[j]>logits[cls])cls=j;
        int unique=1;for(int j=0;j<4;j++)if(j!=cls&&logits[j]>=logits[cls])unique=0;
        enum adas_light predicted=cls==0?LIGHT_RED:cls==1?LIGHT_YELLOW:cls==2?LIGHT_GREEN:LIGHT_UNKNOWN;
        if(!unique||predicted!=candidate->candidate) {if(!allow_pixel)continue;predicted=candidate->candidate;}
        if(candidate->confidence>best->confidence) {
            if(best->candidate!=predicted)*rival=fmaxf(*rival,best->confidence);
            *best=*candidate;
        } else if(best->candidate!=predicted)*rival=fmaxf(*rival,candidate->confidence);
    }
}
void adas_signal_detect(struct signal_detector *s,const uint8_t *image,const struct features *f,
    const struct detection *objects,int count,const struct signal_roi *roi,double arrived,struct adas_signal *out) {
    memset(out,0,sizeof(*out)); out->source=roi->enabled?SIGNAL_ROI:SIGNAL_SEARCH;
    int x0=roi->enabled?roi->x:(roi->intersection==2?720:192),y0=roi->enabled?roi->y:60;
    int width=roi->enabled?roi->w:(roi->intersection==2?480:1536),height=roi->enabled?roi->h:(roi->intersection==2?435:480);
    if(f->source_kind&ADAS_SOURCE_QUAD) {
        int end=y0+height;
        int front_y=adas_quad_front_y(f->source_kind);
        if(y0<front_y)y0=front_y;
        height=end-y0;
    }
    int w=(width+1)/2,h=(height+1)/2;
    if(x0<0||y0<0||width<4||height<4||x0+width>ADAS_W||y0+height>ADAS_H||w*h>GRID_PIXELS) {
        s->hits=0; memset(&s->pending,0,sizeof(s->pending)); return;
    }
    for(int y=0;y<h;y++) for(int x=0;x<w;x++) {
        int r,g,b; channels(image,x0+x*2,y0+y*2,&r,&g,&b); s->mask[y*w+x]=(uint8_t)color(r,g,b,1,roi->intersection?2:1);
    }
    struct adas_signal best={.source=out->source}; float rival=0;
    if(roi->enabled==SIGNAL_ROI_INT8) {
        uint8_t pixels[32*32*3];int32_t logits[4];
        if(width<=128&&height<=128&&light_color_max_input(image,x0,y0,x0+width,y0+height,pixels)) {
            light_color_max_infer(pixels,logits);
            int cls=0,second=-1;
            for(int k=1;k<4;k++)if(logits[k]>logits[cls])cls=k;
            for(int k=0;k<4;k++)if(k!=cls&&(second<0||logits[k]>logits[second]))second=k;
            enum adas_light predicted=cls==0?LIGHT_RED:cls==1?LIGHT_YELLOW:cls==2?LIGHT_GREEN:LIGHT_UNKNOWN;
            int colored=0,matching=0;
            for(int k=0;k<32*32;k++) {
                unsigned type=color(pixels[k*3],pixels[k*3+1],pixels[k*3+2],1,1);
                colored+=type!=LIGHT_UNKNOWN;matching+=type==(unsigned)predicted&&type!=LIGHT_UNKNOWN;
            }
            /* Color support is a consistency score, not a calibrated probability. */
            if(predicted!=LIGHT_UNKNOWN&&logits[cls]>logits[second]&&matching>=3) {
                best=(struct adas_signal){predicted,SIGNAL_ROI,
                    {1,(unsigned)x0,(unsigned)y0,(unsigned)(x0+width-1),(unsigned)(y0+height-1),
                    (unsigned)(x0+width/2),(unsigned)(y0+height/2),0},(float)matching/colored,0};
            }
        }
    } else for(int i=0;i<w*h;i++) if(s->mask[i]) {
        unsigned type=s->mask[i]; int begin=0,end=1,minx=i%w,maxx=minx,miny=i/w,maxy=miny;
        s->queue[0]=i; s->mask[i]=0;
        while(begin<end) {
            int v=s->queue[begin++],x=v%w,y=v/w;
            if(x<minx) minx=x;
            if(x>maxx) maxx=x;
            if(y<miny) miny=y;
            if(y>maxy) maxy=y;
            for(int dy=-1;dy<=1;dy++) for(int dx=-1;dx<=1;dx++) {
                int xx=x+dx,yy=y+dy;
                if(xx<0||xx>=w||yy<0||yy>=h) continue;
                int at=yy*w+xx;
                if(s->mask[at]==type) { s->mask[at]=0; s->queue[end++]=at; }
            }
        }
        int bw=maxx-minx+1,bh=maxy-miny+1;
        float ratio=(float)bw/bh,fill=(float)end/(bw*bh);
        if(end<5||bw<3||bh<3||bw>45||bh>45||ratio<.55f||ratio>1.8f||fill<.45f||fill>.96f) continue;
        int bx0=x0+minx*2,by0=y0+miny*2,bx1=x0+maxx*2+1,by1=y0+maxy*2+1;
        float cx=(bx0+bx1)*.5f,cy=(by0+by1)*.5f; int on_vehicle=0;
        for(int j=0;j<count;j++) if((roi->intersection==2?(objects[j].cls>=0&&objects[j].cls<3):(objects[j].cls<5))&&
            cx>=objects[j].x0&&cx<=objects[j].x1&&
            cy>=objects[j].y0-(roi->intersection==2?(objects[j].y1-objects[j].y0)*.15f:0.f)&&cy<=objects[j].y1) on_vehicle=1;
        if(on_vehicle) continue;
        if(!housing(image,bx0,by0,bx1,by1,type)&&!(roi->intersection==2&&!roi->enabled&&bw<=8&&bh<=8)) continue;
        float surround=dark_surround(image,bx0,by0,bx1,by1);
        if(surround<.30f && !(roi->intersection==2&&!roi->enabled&&surround>=.04f)) continue;
        float score=.45f*surround+.35f*(1-fabsf(1-ratio))+.20f*fminf(1,end/35.f);
        /* Frozen offline ROI model confirms borderline rule candidates only.
         * It supplies neither localization nor a calibrated probability. */
        if(score>=.55f&&score<.75f&&!(roi->intersection==2&&!roi->enabled&&bw<=8&&bh<=8)) {
            uint8_t pixels[32*32*3];int32_t logits[4];
            if(!light_color_max_input(image,bx0,by0,bx1+1,by1+1,pixels))continue;
            light_color_max_infer(pixels,logits);
            int best_class=0;
            for(int k=1;k<4;k++)if(logits[k]>logits[best_class])best_class=k;
            int second_class=best_class==0?1:0;
            for(int k=0;k<4;k++)if(k!=best_class&&logits[k]>logits[second_class])second_class=k;
            enum adas_light predicted=best_class==0?LIGHT_RED:best_class==1?LIGHT_YELLOW:best_class==2?LIGHT_GREEN:LIGHT_UNKNOWN;
            if(predicted!=(enum adas_light)type||(int64_t)logits[best_class]-logits[second_class]<1)continue;
        }
        if(roi->intersection==2&&!roi->enabled&&bw<=8&&bh<=8) {
            score=fmaxf(score,.71f);
            if(s->pending.box.valid&&s->pending.candidate==(enum adas_light)type&&
               fabsf(cx-s->pending.box.cx)<25&&fabsf(cy-s->pending.box.cy)<25)score=fminf(1.f,score+.2f);
        }
        if(score>best.confidence) {
            if(best.candidate!=type&&best.confidence>rival) rival=best.confidence;
            best=(struct adas_signal){(enum adas_light)type,out->source,{1,(unsigned)bx0,(unsigned)by0,(unsigned)bx1,(unsigned)by1,(unsigned)cx,(unsigned)cy,0},score,0};
        } else if(best.candidate!=type&&score>rival) rival=score;
    }
    /* Only this quad clip's explicitly assigned signal corridor enables its small-lamp profile. */
    int quad_video=(f->source_kind&ADAS_SOURCE_QUAD)&&(f->source_kind&ADAS_SOURCE_QUAD_QUARTER)&&
        roi->enabled==SIGNAL_ROI_RULES&&roi->x==840&&roi->y==270&&roi->w==240&&roi->h==210;
    if((quad_video||(!roi->enabled&&roi->intersection==2))&&best.confidence<.70f)
        quad_video_signal(s,image,x0,y0,w,h,objects,count,arrived,&best,&rival,!quad_video);
    if(best.confidence<.70f||rival>best.confidence*.8f) { best.candidate=LIGHT_UNKNOWN; best.box.valid=0; }
    if(roi->intersection==2&&!roi->enabled&&!best.box.valid&&s->hits>=4&&s->pending.box.valid&&
       arrived>s->last_time&&arrived-s->last_time<.25&&rival<.56f) {
        struct candidate b=s->pending.box;
        struct detection box={(float)b.x0-3,(float)b.y0-3,(float)b.x1+3,(float)b.y1+3,0,0};
        int blocked=0,red=0,green=0;
        for(int j=0;j<count;j++)if(objects[j].cls>=0&&objects[j].cls<3&&
           b.cx>=objects[j].x0&&b.cx<=objects[j].x1&&
           b.cy>=objects[j].y0-(objects[j].y1-objects[j].y0)*.15f&&b.cy<=objects[j].y1)blocked=1;
        if(!blocked&&pixel_light_color(image,&box,&red,&green)==s->pending.candidate) {
            best=s->pending;best.source=SIGNAL_SEARCH;
        }
    }
    uint32_t delta=f->source_id-s->last_source;
    int close=best.candidate==s->pending.candidate&&best.candidate!=LIGHT_UNKNOWN&&
        abs((int)best.box.cx-(int)s->pending.box.cx)<(int)(best.box.x1-best.box.x0+12)&&
        abs((int)best.box.cy-(int)s->pending.box.cy)<(int)(best.box.y1-best.box.y0+12)&&
        arrived-s->last_time>0&&arrived-s->last_time<.5&&delta>0&&delta<=15;
    if(!close) { s->hits=best.candidate!=LIGHT_UNKNOWN; s->since=arrived; }
    else if(s->hits<1000) s->hits++;
    s->pending=best; s->last_time=arrived; s->last_source=f->source_id;
    best.stable=s->hits>=4&&arrived-s->since>=.25;
    if(!best.stable) { best.candidate=LIGHT_UNKNOWN; best.box.valid=0; }
    *out=best;
}
