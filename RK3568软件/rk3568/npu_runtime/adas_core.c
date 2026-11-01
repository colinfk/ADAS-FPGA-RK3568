#include "adas_core.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
static unsigned u16(const uint8_t *p) { return p[0]|(unsigned)p[1]<<8; }
static uint32_t u32(const uint8_t *p) { return u16(p)|(uint32_t)u16(p+2)<<16; }
uint32_t adas_xor(const uint8_t *p) {
    uint32_t sum=0;
    for(size_t i=0;i<IMAGE_BYTES;i+=4) sum^=u32(p+i);
    return sum;
}
uint32_t adas_packet_checksum(const uint8_t *p,unsigned long captured,int fast) {
    /* Opt-in sampling only; all trailer structure/freshness checks remain per-frame. */
    return !fast||captured%30==0?adas_xor(p):u32(p+IMAGE_BYTES+72);
}
int adas_parse(const uint8_t *p,struct features *f,uint32_t checksum) {
    const uint8_t *t=p+IMAGE_BYTES;
    memset(f,0,sizeof(*f));
    if(memcmp(t,"FEAT",4)||u16(t+8)!=ADAS_W||u16(t+10)!=ADAS_H||
       t[12]!=1||!(t[13]&8)||u16(t+14)!=64) return -1;
    f->source_id=u32(t+4); f->layout=u32(t+76); f->checksum=checksum;
    if(memcmp(t+64,"FVID",4)||u32(t+68)!=f->source_id||u32(t+72)!=checksum||
       (f->layout!=0x10100&&f->layout!=0x20100)) return -1;
    f->mode=u32(t+88); f->profile=u16(t+92); f->source_kind=u16(t+94);
    if(memcmp(t+80,"LUXF",4)||u32(t+84)!=1||f->mode>3||f->profile>3||f->source_kind>1) return -1;
    for(int i=0;i<3;i++) {
        const uint8_t *r=t+16+i*16;
        struct candidate *c=&f->candidates[i];
        *c=(struct candidate){r[1],u16(r+2),u16(r+4),u16(r+6),u16(r+8),u16(r+10),u16(r+12),r[14]};
        if(r[0]!=i+1||c->valid!=!!(t[13]&(1<<i))) return -1;
        if(c->valid&&(c->x0>c->x1||c->x1>=ADAS_W||c->y0>c->y1||c->y1>=ADAS_H||
           c->cx<c->x0||c->cx>c->x1||c->cy<c->y0||c->cy>c->y1)) return -1;
    }
    unsigned padding=96;
    if(f->layout==0x20100) {
        const uint8_t *r=t+96;
        if(memcmp(r,"RFX1",4)||u32(r+4)!=f->source_id||u32(r+8)!=1||u32(r+12)!=0x7f||
           u32(r+16)!=ADAS_W*ADAS_H||u32(r+96)!=0x2db44060||
           u32(r+100)!=((ADAS_W/2u<<16)|(ADAS_H*2/3u))||
           u32(r+104)!=((ADAS_H<<16)|(ADAS_H/3u))||u32(r+108)!=((ADAS_H<<16)|ADAS_W)||
           u32(r+112)!=(ADAS_W-2)*(ADAS_H-2)||u32(r+116)!=(ADAS_W-4)*(ADAS_H-4)||
           u32(r+120)||u32(r+124)) return -1;
        for(int i=0;i<20;i++) f->road[i]=u32(r+16+4*i);
        padding=224;
    }
    for(unsigned i=padding;i<256;i++) if(t[i]) return -1;
    return 0;
}
void adas_preprocess(const uint8_t *image,uint8_t *rgb) {
    memset(rgb,114,640*640*3);
    /* 1920x1080 -> 640x360 INTER_LINEAR half-pixel centers are (3*x+1,3*y+1).
     * Decode RGB565 as OpenCV does (zero low bits), matching calibration. */
    for(unsigned y=0;y<360;y++) for(unsigned x=0;x<640;x++) {
        unsigned p=u16(image+((3*y+1)*ADAS_W+3*x+1)*2);
        uint8_t *d=rgb+((y+140)*640+x)*3;
        d[0]=(p>>11)<<3; d[1]=((p>>5)&63)<<2; d[2]=(p&31)<<3;
    }
}
/* Demo crop for the supplied red-wait video, in original 1920x1080 coordinates. */
void adas_preprocess_signal_zoom(const uint8_t *image,uint8_t *rgb) {
    memset(rgb,114,640*640*3);
    for(int y=0;y<360;y++)for(int x=0;x<640;x++) {
        float fx=(x+.5f)*240.f/640.f-.5f,fy=(y+.5f)*135.f/360.f-.5f;
        int x0=(int)floorf(fx),y0=(int)floorf(fy);float ax=fx-x0,ay=fy-y0;
        if(x0<0){x0=0;ax=0;} if(y0<0){y0=0;ay=0;}
        int x1=x0<239?x0+1:x0,y1=y0<134?y0+1:y0;
        if(x0>239)x0=x1=239;
        if(y0>134)y0=y1=134;
        unsigned v[4]={u16(image+((360+y0)*ADAS_W+840+x0)*2),u16(image+((360+y0)*ADAS_W+840+x1)*2),
                       u16(image+((360+y1)*ADAS_W+840+x0)*2),u16(image+((360+y1)*ADAS_W+840+x1)*2)};
        uint8_t *q=rgb+((y+140)*640+x)*3;
        for(int c=0;c<3;c++) {
            int shift=c==0?11:c==1?5:0,mask=c==1?63:31,mul=c==1?4:8;
            float a=((v[0]>>shift)&mask)*mul,b=((v[1]>>shift)&mask)*mul;
            float d=((v[2]>>shift)&mask)*mul,e=((v[3]>>shift)&mask)*mul;
            q[c]=(uint8_t)lroundf((a+(b-a)*ax)*(1-ay)+(d+(e-d)*ax)*ay);
        }
    }
}
void adas_preprocess_signal_wide(const uint8_t *image,uint8_t *rgb) {
    memset(rgb,114,640*640*3);
    for(int y=0;y<360;y++)for(int x=0;x<640;x++) {
        float fx=(x+.5f)*1.5f-.5f,fy=(y+.5f)*1.5f-.5f;
        int x0=(int)floorf(fx),y0=(int)floorf(fy);float ax=fx-x0,ay=fy-y0;
        int x1=x0<959?x0+1:x0,y1=y0<539?y0+1:y0;
        unsigned v[4]={u16(image+((180+y0)*ADAS_W+480+x0)*2),u16(image+((180+y0)*ADAS_W+480+x1)*2),
                       u16(image+((180+y1)*ADAS_W+480+x0)*2),u16(image+((180+y1)*ADAS_W+480+x1)*2)};
        uint8_t *q=rgb+((y+140)*640+x)*3;
        for(int c=0;c<3;c++) {
            int shift=c==0?11:c==1?5:0,mask=c==1?63:31,mul=c==1?4:8;
            float a=((v[0]>>shift)&mask)*mul,b=((v[1]>>shift)&mask)*mul;
            float d=((v[2]>>shift)&mask)*mul,e=((v[3]>>shift)&mask)*mul;
            q[c]=(uint8_t)lroundf((a+(b-a)*ax)*(1-ay)+(d+(e-d)*ax)*ay);
        }
    }
}
static float clamp(float x,float lo,float hi) { return fmaxf(lo,fminf(x,hi)); }
static float overlap(const struct detection *a,const struct detection *b) {
    float w=fmaxf(0,fminf(a->x1,b->x1)-fmaxf(a->x0,b->x0));
    float h=fmaxf(0,fminf(a->y1,b->y1)-fmaxf(a->y0,b->y0));
    float aa=(a->x1-a->x0)*(a->y1-a->y0),bb=(b->x1-b->x0)*(b->y1-b->y0);
    return w*h/fmaxf(aa+bb-w*h,1e-6f);
}
static int compare(const void *a,const void *b) {
    float x=((const struct detection *)a)->score,y=((const struct detection *)b)->score;
    return (y>x)-(y<x);
}
static float model10_class_thr(int raw_cls,float confidence) {
    /* Mother display/road_user already gate at .35; raise vehicle/ped decode floor to match.
     * Traffic light (mapped 6) stays at the CLI confidence so small/far lights remain visible. */
    static const int mapping[10]={0,1,2,-1,-1,-1,5,-1,-1,6};
    int mapped=raw_cls>=0&&raw_cls<10?mapping[raw_cls]:-1;
    if(mapped>=0&&mapped<=2) return fmaxf(confidence,.35f); /* car/truck/bus */
    if(mapped==5) return fmaxf(confidence,.35f); /* pedestrian */
    return confidence; /* light and anything else */
}
static int adas_decode_impl(const float *o[9],float confidence,float iou,struct detection out[MAX_DETECTIONS],int model10) {
    static const int mapping[10]={0,1,2,-1,-1,-1,5,-1,-1,6};
    int classes=model10?10:6;
    /* Keep every multi-label candidate before class-wise NMS; at most 8400*6. */
    struct detection *all=malloc(8400*7*sizeof(*all));
    if(!all) return -1;
    int n=0;
    for(int branch=0;branch<3;branch++) {
        int stride=8<<branch,w=640/stride,area=w*w;
        for(int i=0;i<area;i++) {
            int eligible=0;
            for(int cls=0;cls<classes;cls++) {
                if(model10&&mapping[cls]<0)continue;
                float score=o[branch*3+1][cls*area+i];
                if(!isfinite(score)||score<-.01f||score>1.01f) { free(all); return -1; }
                float thr=model10?model10_class_thr(cls,confidence):confidence;
                eligible|=score>=thr;
            }
            /* Do not gate on separately quantized class-sum: rounding may hide a threshold hit. */
            if(!eligible) continue;
            float distance[4];
            for(int side=0;side<4;side++) {
                float max=-INFINITY,total=0,value=0;
                for(int k=0;k<16;k++) {
                    float logit=o[branch*3][(side*16+k)*area+i];
                    if(!isfinite(logit)) { free(all); return -1; }
                    max=fmaxf(max,logit);
                }
                for(int k=0;k<16;k++) {
                    float e=expf(o[branch*3][(side*16+k)*area+i]-max);
                    total+=e; value+=k*e;
                }
                distance[side]=value/total;
            }
            float cx=i%w+.5f,cy=i/w+.5f;
            for(int cls=0;cls<classes;cls++) {
                if(model10&&mapping[cls]<0)continue;
                float score=o[branch*3+1][cls*area+i];
                float thr=model10?model10_class_thr(cls,confidence):confidence;
                if(score<thr) continue;
                all[n++]=(struct detection){(cx-distance[0])*stride,(cy-distance[1])*stride,
                    (cx+distance[2])*stride,(cy+distance[3])*stride,score,model10?mapping[cls]:cls};
            }
        }
    }
    qsort(all,n,sizeof(*all),compare);
    struct detection selected[MAX_DETECTIONS]; int count=0;
    for(int i=0;i<n&&count<MAX_DETECTIONS;i++) {
        int keep=1;
        for(int j=0;j<count;j++) if(all[i].cls==selected[j].cls&&overlap(&all[i],&selected[j])>iou) { keep=0; break; }
        if(keep) selected[count++]=all[i];
    }
    for(int i=0;i<count;i++) {
        out[i]=selected[i];
        out[i].x0=clamp(selected[i].x0*3,0,ADAS_W-1); out[i].x1=clamp(selected[i].x1*3,0,ADAS_W-1);
        out[i].y0=clamp((selected[i].y0-140)*3,0,ADAS_H-1); out[i].y1=clamp((selected[i].y1-140)*3,0,ADAS_H-1);
    }
    free(all); return count;
}
int adas_decode(const float *o[9],float confidence,float iou,struct detection out[MAX_DETECTIONS]) {return adas_decode_impl(o,confidence,iou,out,0);}
int adas_decode10(const float *o[9],float confidence,float iou,struct detection out[MAX_DETECTIONS]) {return adas_decode_impl(o,confidence,iou,out,1);}
static int road_gray(const uint8_t *image,int x,int y) {
    unsigned p=u16(image+(y*ADAS_W+x)*2);
    return (int)((((p>>11)<<3)+2*(((p>>5)&63)<<2)+((p&31)<<3))/4);
}
static int marking(const uint8_t *image,int x,int y) {
    unsigned p=u16(image+(y*ADAS_W+x)*2),r=(p>>11)<<3,g=((p>>5)&63)<<2,b=(p&31)<<3;
    unsigned min=r<g?r:g,max=r>g?r:g;
    min=min<b?min:b; max=max>b?max:b;
    return (min>=100&&max-min<=48)||(r>=128&&g>=112&&b+48<g&&r+32>=g);
}
static int occluded(const struct detection *objects,int count,float x,float y) {
    for(int i=0;i<count;i++) if(objects[i].score>=.35f&&x>=objects[i].x0-4&&x<=objects[i].x1+4&&
        y>=objects[i].y0-4&&y<=objects[i].y1+4) return 1;
    return 0;
}
#define ROW_MARKS 24
struct road_row { float centers[ROW_MARKS]; unsigned count; };
static int row_match(const struct road_row *row,float predicted,float tolerance) {
    int best=-1; float distance=tolerance;
    for(unsigned j=0;j<row->count;j++) {
        float delta=fabsf(row->centers[j]-predicted);
        if(delta<distance) { best=(int)j; distance=delta; }
    }
    return best;
}
static int natural_color(const uint8_t *image,int x,int y) {
    unsigned p=u16(image+(y*ADAS_W+x)*2);
    int r=(p>>11)<<3,g=((p>>5)&63)<<2,b=(p&31)<<3;
    int low=r<g?r:g,high=r>g?r:g;low=low<b?low:b;high=high>b?high:b;
    if(r>=115&&g>=95&&r>=b+28&&g>=b+18)return 2;
    return low>=90&&high-low<=55;
}
static int natural_yellow(const uint8_t *image,int x,int y) {
    unsigned pixel=u16(image+(y*ADAS_W+x)*2);
    int red=(pixel>>11)<<3,green=((pixel>>5)&63)<<2,blue=(pixel&31)<<3;
    return red>135&&green>115&&blue<110&&red+green-2*blue>90;
}
static int natural_chroma(const uint8_t *image,int x,int y) {
    unsigned pixel=u16(image+(y*ADAS_W+x)*2);
    return (int)((pixel>>11)<<3)+(int)(((pixel>>5)&63)<<2)-2*(int)((pixel&31)<<3);
}
static int natural_marking(const uint8_t *image,int x,int y) {
    int color=natural_color(image,x,y);
    if(!color)return 0;
    if(color==2&&natural_chroma(image,x,y)>=
       (natural_chroma(image,x-8,y)+natural_chroma(image,x+8,y))/2+18)return 1;
    if(natural_yellow(image,x,y)&&
       (!natural_yellow(image,x-8,y)||!natural_yellow(image,x+8,y)))return 1;
    int background=(road_gray(image,x-8,y)+road_gray(image,x+8,y))/2;
    return road_gray(image,x,y)>=background+(color==2?12:14);
}
/* One row-cluster/vote/least-squares engine; strict mode retains legacy arithmetic. */
static void fit_lane_core(const uint8_t *image,const struct candidate *c,int side,
    const struct detection *objects,int count,struct lane *lane,int y_min,int y_max,int natural) {
    enum { ROWS=129 };
    int quad=natural&&y_min==QUAD_ROAD_MIN&&y_max==QUAD_ROAD_MAX;
    int step=natural&&y_max<=760?2:4,scale=4/step,anchor=natural?(y_max<=760?y_max:y_max+4):1076;
    int row_count=(y_max-y_min)/step+1,min_span=natural?(y_min==660?80:48)/step:40;
    int minimum=natural?10*scale:18,lo=side?(natural?990:976):(natural?140:80);
    int hi=side?(natural?1780:1840):(natural?930:944);
    struct road_row rows[ROWS]={0};
    for(int row=0;row<row_count;row++) {
        int y=y_min+row*step;
        for(int x=lo;x<=hi;) {
            int start=x,center;
            if(natural) {
                if(!natural_marking(image,x,y)){x++;continue;}
                int seed=x,edge_background=road_gray(image,start-8,y);
                /* Follow paint ridges without letting a broad pavement edge consume them. */
                int yellow_band=natural_color(image,x,y)==2,edge_chroma=natural_chroma(image,start-8,y);
                while(start>lo&&(yellow_band?(natural_color(image,start-1,y)==2&&natural_chroma(image,start-1,y)>=edge_chroma+12):
                      natural_color(image,start-1,y)&&road_gray(image,start-1,y)>=edge_background+8))start--;
                while(x<=hi&&(yellow_band?(natural_color(image,x,y)==2&&natural_chroma(image,x,y)>=edge_chroma+12):
                      natural_color(image,x,y)&&road_gray(image,x,y)>=edge_background+8))x++;
                if(x==seed){x++;continue;}
                int width=x-start;center=(start+x-1)/2;
                /* Near quad-road paint widens in perspective; retain the far-field transverse-line cap. */
                if(width>26+(quad&&y>704?(y-704)/12:0)){x=seed+1;continue;}
                if(width<2||occluded(objects,count,center,y))continue;
                int before=road_gray(image,start-8,y),after=road_gray(image,x+8,y);
                if(yellow_band) {
                    if(natural_chroma(image,center,y)<(natural_chroma(image,start-8,y)+natural_chroma(image,x+8,y))/2+18)continue;
                } else {
                    if(abs(before-after)>40||road_gray(image,center,y)<before+8||road_gray(image,center,y)<after+8)continue;
                    if(road_gray(image,center,y)<(before+after)/2+(natural_color(image,center,y)==2?12:14))continue;
                }
            } else {
                if(!marking(image,x,y)){x++;continue;}
                while(x<=hi&&marking(image,x,y))x++;
                int width=x-start;center=(start+x-1)/2;
                if(width<2||width>12+(y-600)/8||occluded(objects,count,center,y))continue;
                int before=start-6,after=x+6;
                if(before<0||after>=ADAS_W)continue;
                if(road_gray(image,center,y)<(road_gray(image,before,y)+road_gray(image,after,y))/2+20)continue;
            }
            if(rows[row].count<ROW_MARKS)rows[row].centers[rows[row].count++]=center;
        }
    }
    unsigned present=0;for(int row=0;row<row_count;row++)present+=rows[row].count>0;
    if(present<(unsigned)minimum)return;
    float best=0,a=0,b=0,tolerance=natural?10:14;
    int bins=natural?100:50,spacing=natural?8:16,start=natural?lo:(side?976:160);
    for(int angle=natural?7:2;angle<=(natural?47:36);angle++) {
        float slope=(side?1.f:-1.f)*angle*.05f;
        unsigned votes[100]={0};int first[100]={0},last[100]={0};
        for(int row=0;row<row_count;row++) {
            unsigned char seen[100]={0};
            for(unsigned j=0;j<rows[row].count;j++) {
                float bottom=rows[row].centers[j]+slope*(anchor-(y_min+row*step));
                float position=(bottom-start)/(float)spacing;
                int bin=natural?(int)lroundf(position):(int)floorf(position+.5f);
                for(int i=bin-1;i<=bin+1;i++) {
                    if(i<0||i>=bins||seen[i]||fabsf(bottom-(start+i*spacing))>=tolerance)continue;
                    seen[i]=1;if(!votes[i])first[i]=row;votes[i]++;last[i]=row;
                }
            }
        }
        for(int i=0;i<bins;i++) {
            float bottom=start+i*spacing,intercept=bottom-slope*anchor,top=slope*y_min+intercept;
            if(votes[i]<(unsigned)minimum||last[i]-first[i]<min_span)continue;
            if(natural) {
                float horizon=(960-intercept)/slope;
                if(bottom>hi||horizon<280||horizon>fminf(736,y_max-12)||top<400||top>1520)continue;
            } else if(top<480||top>1440||fabsf(top-960)>fabsf(bottom-960)-40)continue;
            float score=votes[i]*(1.f-(natural?.25f:.35f)*fabsf(bottom-960)/960.f);
            if(!natural&&c&&c->valid&&c->rows>=12&&c->rows<=128&&c->x1>c->x0&&
               c->x1-c->x0<ADAS_W/2&&fabsf(bottom-(float)c->cx)<120.f)score*=1.12f;
            if(score>best){best=score;a=slope;b=intercept;}
        }
    }
    if(!best)return;
    for(int pass=0;pass<2;pass++) {
        double sx=0,sy=0,sxy=0,syy=0;float fx=0,fy=0,fxy=0,fyy=0;int used=0;
        for(int row=0;row<row_count;row++) {
            float y=y_min+row*step;int i=row_match(&rows[row],a*y+b,pass?10:tolerance);
            if(i<0)continue;
            float x=rows[row].centers[i];used++;
            if(natural){sx+=x;sy+=y;sxy+=x*y;syy+=y*y;}
            else {fx+=x;fy+=y;fxy+=x*y;fyy+=y*y;}
        }
        if(used<minimum)return;
        if(natural) {
            double denominator=used*syy-sy*sy;if(denominator<1)return;
            a=(float)((used*sxy-sx*sy)/denominator);b=(float)((sx-a*sy)/used);
        } else {
            float denominator=used*fyy-fy*fy;if(denominator<1)return;
            a=(used*fxy-fx*fy)/denominator;b=(fx-a*fy)/used;
        }
    }
    if((side?a:-a)<(natural?.35f:.1f)||fabsf(a)>(natural?2.35f:1.8f))return;
    float horizon=(960-b)/a;
    if(natural&&(horizon<280||horizon>fminf(736,y_max-12)))return;
    unsigned char hit[ROWS]={0},blocked[ROWS]={0};
    int used=0,visible=0,first=-1,last=-1,yellow=0,hood=0,far=0;float residual=0;
    for(int row=0;row<row_count;row++) {
        float y=y_min+row*step,x=a*y+b;
        if(x<lo||x>hi||occluded(objects,count,x,y)){blocked[row]=1;continue;}
        visible++;int i=row_match(&rows[row],x,10);
        if(i>=0) {
            hit[row]=1;if(first<0)first=row;last=row;used++;
            residual+=fabsf(rows[row].centers[i]-x);
            if(natural){hood+=y>=850;far+=y<=846;yellow+=natural_yellow(image,(int)rows[row].centers[i],(int)y);}
        }
    }
    if(visible<(natural?(y_min==660?30:20)*scale:72)||used<minimum||last-first<min_span||residual/used>(natural?10:6))return;
    if(natural&&y_max>900&&hood*10>used*7&&far<4){memset(lane,0,sizeof(*lane));return;}
    if(!natural)for(int row=1;row<row_count-1;row++)if(!blocked[row]&&hit[row-1]&&hit[row+1])hit[row]=1;
    int total=0,longest_gap=0,gap=0,segments=0,run=0,internal=0;
    for(int row=0;row<row_count;row++) {
        total+=hit[row];if(blocked[row]){gap=run=0;continue;}
        if(hit[row]) {
            if(gap>=5&&row>first&&row<=last)internal++;
            if(!natural&&gap>longest_gap)longest_gap=gap;
            gap=0;run++;if(run==(natural?3*scale:4))segments++;
        } else {run=0;gap++;if(natural&&gap>longest_gap&&(!quad||(row>first&&row<last)))longest_gap=gap;}
    }
    /* Surround fits use a wide road ROI: unseen leading/trailing rows are
     * not a dashed-line gap between two observed paint segments. */
    if(!quad&&gap>longest_gap)longest_gap=gap;
    float coverage=(float)total/visible;enum lane_type type=LANE_UNKNOWN;
    if(natural) {
        if((coverage>=.48f&&longest_gap<=5*scale)||((float)yellow/used>=.35f&&coverage>=.42f))type=LANE_SOLID;
        else if(segments>=2&&longest_gap>=4*scale&&coverage>=.12f&&coverage<=.64f)type=LANE_DASHED;
    } else {
        if(coverage>=.84f&&longest_gap<=4)type=LANE_SOLID;
        else if(coverage>=.18f&&coverage<=.72f&&segments>=2&&internal>=1&&last-first>=55)type=LANE_DASHED;
    }
    if(!natural||type!=LANE_UNKNOWN||!lane->valid)*lane=(struct lane){a,b,coverage,residual/used,1,type};
}
void adas_lane_reject_hood(const uint8_t *image,const struct detection *objects,int count,struct lane *lane) {
    if(!lane->valid)return;
    int near=0,far=0;
    for(int y=660;y<=1032;y+=4) {
        int center=(int)lroundf(lane->slope*y+lane->intercept);
        if(center<24||center>ADAS_W-24||occluded(objects,count,(float)center,(float)y))continue;
        int hit=0;
        for(int dx=-8;dx<=8;dx++)hit|=natural_marking(image,center+dx,y);
        if(hit){near+=y>=850;far+=y<=846;}
    }
    if(near*10>(near+far)*7&&far<4)memset(lane,0,sizeof(*lane));
}
void adas_lane_masked(const uint8_t *image,const struct candidate *c,int side,
                      const struct detection *objects,int count,struct lane *lane) {
    memset(lane,0,sizeof(*lane));
    if(side<0||side>1)return;
    fit_lane_core(image,c,side,objects,count,lane,600,1076,0);
    if(lane->valid&&lane->type!=LANE_UNKNOWN)return;
    fit_lane_core(image,NULL,side,objects,count,lane,660,1032,1);
}
void adas_lane_quad_masked(const uint8_t *image,int side,
    const struct detection *objects,int count,struct lane *lane) {
    memset(lane,0,sizeof(*lane));
    if(side<0||side>1)return;
    fit_lane_core(image,NULL,side,objects,count,lane,QUAD_ROAD_MIN,QUAD_ROAD_MAX,1);
}
void adas_lane(const uint8_t *image,const struct candidate *c,int side,struct lane *lane) {
    adas_lane_masked(image,c,side,NULL,0,lane);
}
int adas_crosswalk_confirm(const uint8_t *image,const struct candidate *c) {
    if(!image||!c||!c->valid)return 0;
    /* Reject multiple longitudinal stripes: require three separated long,
     * bright horizontal bars in the central road region. Candidate only. */
    int bands=0,in_band=0,first=-1,last=-1,last_bar=-100;
    for(int y=540;y<=1020;y+=4) {
        int run=0,longest=0;
        for(int x=576;x<1344;x+=2) {
            const uint8_t *p=image+(y*ADAS_W+x)*2;unsigned v=p[0]|(unsigned)p[1]<<8;
            int r=(int)(v>>11)<<3,g=(int)((v>>5)&63)<<2,b=(int)(v&31)<<3;
            int lo=r<g?r:g;lo=lo<b?lo:b;int hi=r>g?r:g;hi=hi>b?hi:b;
            if(lo>=180&&hi-lo<=45)run+=2;else run=0;
            if(run>longest)longest=run;
        }
        if(longest>=160) {
            if(!in_band&&y-last_bar>=8){bands++;if(first<0)first=y;last=y;}
            in_band=1;last_bar=y;
        } else in_band=0;
    }
    return bands>=3&&last-first>=40;
}
void adas_lane_geometry_stabilize(struct lane_geometry_history *h,struct lane *lane,int reset) {
    if(reset) memset(h,0,sizeof(*h));
    if(!lane->valid||!isfinite(lane->slope)||!isfinite(lane->intercept)||
       !isfinite(lane->residual)||lane->residual>10||fabsf(lane->slope)>2.35f) {
        memset(h,0,sizeof(*h)); memset(lane,0,sizeof(*lane)); return;
    }
    float top=lane->slope*720+lane->intercept,bottom=lane->slope*1076+lane->intercept;
    int same=h->last.valid&&fabsf(top-(h->last.slope*720+h->last.intercept))<32&&
        fabsf(bottom-(h->last.slope*1076+h->last.intercept))<40;
    h->hits=same?(h->hits<3?h->hits+1:3):1;
    if(same) {
        lane->slope=.75f*lane->slope+.25f*h->last.slope;
        lane->intercept=.75f*lane->intercept+.25f*h->last.intercept;
    }
    h->last=*lane; lane->valid=h->hits>=3;
}
/* Hold last stable class across brief UNKNOWN gaps (competition video anti-flicker). */
#define LANE_HOLD_FRAMES 12
void adas_lane_stabilize(struct lane_history *h,struct lane *lane,int reset) {
    if(reset) memset(h,0,sizeof(*h));
    if(!lane->valid||lane->type==LANE_UNKNOWN) {
        h->pending=LANE_UNKNOWN; h->hits=0;
        /* Presentational hold: keep last stable TYPE for HUD capsules briefly.
         * Safety: force valid=0 so geometry()/EVENT_NO_LANE still see lane loss.
         * Do NOT restore stale slope/intercept as a live lane. */
        if(h->stable!=LANE_UNKNOWN&&h->hold_miss<LANE_HOLD_FRAMES) {
            h->hold_miss++;
            lane->type=h->stable;
            lane->valid=0;
            return;
        }
        memset(h,0,sizeof(*h));
        lane->type=LANE_UNKNOWN;
        lane->valid=0;
        return;
    }
    float bottom=lane->slope*1076+lane->intercept,top=lane->slope*720+lane->intercept;
    float old_bottom=h->last.slope*1076+h->last.intercept,old_top=h->last.slope*720+h->last.intercept;
    int consistent=h->last.valid&&fabsf(bottom-old_bottom)<40&&fabsf(top-old_top)<32;
    if(h->pending!=lane->type||!consistent) { h->pending=lane->type; h->hits=1; /* keep prior stable until 3 hits */ }
    else if(h->hits<3) h->hits++;
    if(consistent) {
        lane->slope=.75f*lane->slope+.25f*h->last.slope;
        lane->intercept=.75f*lane->intercept+.25f*h->last.intercept;
    }
    if(h->hits>=3) { h->stable=h->pending; h->hold_miss=0; }
    else if(h->stable!=LANE_UNKNOWN) {
        if(h->hold_miss<LANE_HOLD_FRAMES) h->hold_miss++;
        else {
            h->stable=h->pending=LANE_UNKNOWN;
            h->hits=h->hold_miss=0;
        }
    }
    h->last=*lane;
    /* Prefer confirmed stable; during ramp-up keep previous stable if any (2–3 frame feel). */
    if(h->stable!=LANE_UNKNOWN) lane->type=h->stable;
    else lane->type=LANE_UNKNOWN;
    /* An unconfirmed/held class is presentation data, not a live alert input. */
    if(h->hits<3) lane->valid=0;
}
const char *adas_class_name(int cls) {
    static const char *names[]={"CAR","TRUCK","BUS","MOTORCYCLE","BICYCLE","PERSON"};
    return cls>=0&&cls<6?names[cls]:"UNKNOWN";
}
const char *adas_lane_class_name(enum lane_class cls) {
    static const char *const names[]={"NONE","DASHED","SOLID","INVALID","UNKNOWN"};
    return cls>=LANE_CLASS_NONE&&cls<=LANE_CLASS_UNKNOWN?names[cls]:"INVALID";
}

void adas_quad_features(const uint8_t *image,struct features *f) {
    f->source_kind|=ADAS_SOURCE_QUAD;
    memset(f->candidates,0,sizeof(f->candidates));
    unsigned count=0,sum=0,dark=0;
    for(int y=QUAD_ROAD_MIN;y<=QUAD_ROAD_MAX;y+=4)for(int x=140;x<1780;x+=4) {
        int value=road_gray(image,x,y);count++;sum+=(unsigned)value;dark+=value<40;
    }
    /* Whole-frame FPGA edge counters remain diagnostics; front safety uses this ROI. */
    f->road[0]=count;f->road[1]=sum;f->road[2]=dark;
}
struct quad_roi { int x,y,w,h; };
static struct quad_roi quad_roi(int region,enum quad_layout layout) {
    /* Overlap at the two seams lets C conservatively mark both adjacent modules. */
    static const struct quad_roi regions[]={
        {0,QUAD_FRONT_Y0,1920,ADAS_H-QUAD_FRONT_Y0},
        {0,QUAD_TOP_Y,640,QUAD_MOD_H},{640,QUAD_TOP_Y,640,QUAD_MOD_H},{1280,QUAD_TOP_Y,640,QUAD_MOD_H}};
    static const struct quad_roi quarter[]={
        {0,270,1920,810},{0,0,640,270},{640,0,640,270},{1280,0,640,270}};
    if(layout==QUAD_LAYOUT_REAR_QUARTER)return quarter[region];
    return regions[region];
}
int adas_quad_preprocess_layout(const uint8_t *image,int region,enum quad_layout layout,uint8_t *rgb) {
    if(!image||!rgb||region<0||region>3||layout<0||layout>QUAD_LAYOUT_REAR_QUARTER)return -1;
    struct resize_table {uint16_t x0[640],x1[640],y0[640],y1[640];float wx[640],wy[640];int height,pad,ready;};
    /* One inference worker owns each table; TLS also makes concurrent test callers safe. */
    static _Thread_local struct resize_table tables[2][4];
    struct resize_table *t=&tables[layout][region];
    if(!t->ready) {
        struct quad_roi r=quad_roi(region,layout);
        t->height=(int)lroundf(640.f*r.h/r.w);t->pad=(640-t->height)/2;
        for(int x=0;x<640;x++) {
            float fx=clamp((x+.5f)*r.w/640.f-.5f,0,r.w-1);int xx=(int)fx;
            t->x0[x]=(uint16_t)(r.x+xx);t->x1[x]=(uint16_t)(r.x+(xx+1<r.w?xx+1:r.w-1));t->wx[x]=fx-xx;
        }
        for(int y=0;y<t->height;y++) {
            float fy=clamp((y+.5f)*r.h/t->height-.5f,0,r.h-1);int yy=(int)fy;
            t->y0[y]=(uint16_t)(r.y+yy);t->y1[y]=(uint16_t)(r.y+(yy+1<r.h?yy+1:r.h-1));t->wy[y]=fy-yy;
        }
        t->ready=1;
    }
    memset(rgb,114,640*640*3);
    for(int y=0;y<t->height;y++)for(int x=0;x<640;x++) {
        float dx=t->wx[x],dy=t->wy[y];
        float weights[4]={(1-dx)*(1-dy),dx*(1-dy),(1-dx)*dy,dx*dy};
        unsigned pixels[4]={u16(image+(t->y0[y]*ADAS_W+t->x0[x])*2),u16(image+(t->y0[y]*ADAS_W+t->x1[x])*2),
            u16(image+(t->y1[y]*ADAS_W+t->x0[x])*2),u16(image+(t->y1[y]*ADAS_W+t->x1[x])*2)};
        float values[3]={0};
        for(int i=0;i<4;i++) {
            unsigned p=pixels[i];float weight=weights[i];
            values[0]+=((p>>11)<<3)*weight;values[1]+=(((p>>5)&63)<<2)*weight;values[2]+=((p&31)<<3)*weight;
        }
        for(int c=0;c<3;c++) {
            /* Nonnegative samples: exact half-up rounding without a per-pixel libm call. */
            int whole=(int)values[c];
            rgb[((y+t->pad)*640+x)*3+c]=(uint8_t)(whole+(values[c]-whole>=.5f));
        }
    }
    return 0;
}
int adas_quad_preprocess(const uint8_t *image,int region,uint8_t *rgb) {
    return adas_quad_preprocess_layout(image,region,QUAD_LAYOUT_LEGACY,rgb);
}
int adas_quad_decode_layout(const float *outputs[9],int region,enum quad_layout layout,float confidence,float iou,
                     struct detection detections[MAX_DETECTIONS]) {
    if(region<0||region>3||layout<0||layout>QUAD_LAYOUT_REAR_QUARTER)return -1;
    int n=adas_decode(outputs,confidence,iou,detections),used=0;
    if(n<0)return -1;
    struct quad_roi r=quad_roi(region,layout);int height=(int)lroundf(640.f*r.h/r.w),pad=(640-height)/2;
    for(int i=0;i<n;i++) {
        struct detection d=detections[i];
        d.x0=r.x+clamp(d.x0/3*r.w/640,0,r.w-1);d.x1=r.x+clamp(d.x1/3*r.w/640,0,r.w-1);
        d.y0=r.y+clamp((d.y0/3+140-pad)*r.h/height,0,r.h-1);
        d.y1=r.y+clamp((d.y1/3+140-pad)*r.h/height,0,r.h-1);
        if(d.x1>d.x0+2&&d.y1>d.y0+2)detections[used++]=d;
    }
    return used;
}
int adas_quad_decode(const float *outputs[9],int region,float confidence,float iou,
    struct detection detections[MAX_DETECTIONS]) {
    return adas_quad_decode_layout(outputs,region,QUAD_LAYOUT_LEGACY,confidence,iou,detections);
}
int adas_quad_append(struct detection all[MAX_DETECTIONS],int count,const struct detection *part,int size) {
    if(count<0||count>MAX_DETECTIONS||size<0||size>MAX_DETECTIONS)return -1;
    for(int i=0;i<size&&i<75&&count<MAX_DETECTIONS;i++) {
        int duplicate=0;
        for(int j=0;j<count;j++)if(all[j].cls==part[i].cls&&overlap(&all[j],&part[i])>.5f) {
            if(part[i].score>all[j].score)all[j]=part[i];
            duplicate=1;break;
        }
        if(!duplicate)all[count++]=part[i];
    }
    return count;
}
