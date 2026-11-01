#include "adas_nav.h"
#include "nav_glyphs.h"
#include "adas_state.h"
#include "nav_signal_templates.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

_Static_assert(NAV_GLYPH_W==32&&NAV_GLYPH_H==48,"navigation atlas dimensions");
struct nav_box { int x,y,w,h; };
enum { NAV_DISTANCE_X=109,NAV_DISTANCE_Y=20,NAV_DISTANCE_W=190,NAV_DISTANCE_H=74 };
static unsigned nav_pixel(const uint8_t *im,int x,int y) {
    const uint8_t *p=im+2*(y*ADAS_W+x);return p[0]|(unsigned)p[1]<<8;
}
static int nav_white(const uint8_t *im,int x,int y) {
    unsigned p=nav_pixel(im,x,y);
    int r=(int)(p>>11)*8,g=(int)((p>>5)&63)*4,b=(int)(p&31)*8;
    int lo=r<g?r:g;lo=lo<b?lo:b;
    int hi=r>g?r:g;hi=hi>b?hi:b;
    return hi>=190&&(hi-lo)*255<=95*hi;
}
static int nav_bounds(const uint8_t *im,struct nav_box roi,struct nav_box *out) {
    int x0=roi.x+roi.w,y0=roi.y+roi.h,x1=-1,y1=-1,area=0;
    for(int y=roi.y;y<roi.y+roi.h;y++)for(int x=roi.x;x<roi.x+roi.w;x++)if(nav_white(im,x,y)) {
        if(x<x0)x0=x;
        if(x>x1)x1=x;
        if(y<y0)y0=y;
        if(y>y1)y1=y;
        area++;
    }
    if(!area)return 0;
    *out=(struct nav_box){x0,y0,x1-x0+1,y1-y0+1};return area;
}
static void nav_normalize(const uint8_t *im,struct nav_box box,float mask[NAV_GLYPH_W*NAV_GLYPH_H]) {
    /* Same tight-bbox / aspect-preserving INTER_AREA normalization as Python. */
    memset(mask,0,sizeof(float)*NAV_GLYPH_W*NAV_GLYPH_H);
    float scale=fminf(28.f/box.w,44.f/box.h);
    int width=(int)lrintf(box.w*scale),height=(int)lrintf(box.h*scale);
    if(width<1)width=1;
    if(height<1)height=1;
    int ox=(NAV_GLYPH_W-width)/2,oy=(NAV_GLYPH_H-height)/2;
    for(int y=0;y<height;y++)for(int x=0;x<width;x++) {
        float x0=(float)x*box.w/width,x1=(float)(x+1)*box.w/width;
        float y0=(float)y*box.h/height,y1=(float)(y+1)*box.h/height,total=0;
        for(int sy=(int)y0;sy<(int)ceilf(y1)&&sy<box.h;sy++)for(int sx=(int)x0;sx<(int)ceilf(x1)&&sx<box.w;sx++) {
            float weight=(fminf(x1,sx+1)-fmaxf(x0,sx))*(fminf(y1,sy+1)-fmaxf(y0,sy));
            total+=weight*nav_white(im,box.x+sx,box.y+sy);
        }
        mask[(oy+y)*NAV_GLYPH_W+ox+x]=total/((x1-x0)*(y1-y0))>=100.f/255.f;
    }
}
static float nav_similarity(const float *mask,const unsigned char *glyph,int mirror) {
    float intersection=0,total=0;
    for(int y=0;y<NAV_GLYPH_H;y++)for(int x=0;x<NAV_GLYPH_W;x++) {
        float a=mask[y*NAV_GLYPH_W+x];
        float b=(float)glyph[y*NAV_GLYPH_W+(mirror?NAV_GLYPH_W-1-x:x)];
        intersection+=a*b;total+=a+b;
    }
    return total>0?2*intersection/total:0;
}
static int nav_digit(const uint8_t *im,struct nav_box box,float *confidence) {
    float mask[NAV_GLYPH_W*NAV_GLYPH_H],best=0,second=0,scores[10]={0};int value=-1;
    nav_normalize(im,box,mask);
    for(int i=0;i<NAV_DIGIT_TEMPLATE_COUNT;i++) {
        int label=nav_digit_labels[i];
        if(label<0||label>9)continue;
        float score=nav_similarity(mask,nav_digit_templates[i],0);
        if(score>scores[label])scores[label]=score;
    }
    for(int label=0;label<10;label++) {
        float score=scores[label];
        if(score>best){second=best;best=score;value=label;}
        else if(score>second)second=score;
    }
    *confidence=best;
    return best>=.72f&&best-second>=.035f?value:-1;
}
static int nav_components(const uint8_t *im,struct nav_box boxes[128]) {
    unsigned char pixels[NAV_DISTANCE_W*NAV_DISTANCE_H];
    uint16_t queue[NAV_DISTANCE_W*NAV_DISTANCE_H];
    for(int y=0;y<NAV_DISTANCE_H;y++)for(int x=0;x<NAV_DISTANCE_W;x++)
        pixels[y*NAV_DISTANCE_W+x]=(unsigned char)nav_white(im,x+NAV_DISTANCE_X,y+NAV_DISTANCE_Y);
    int count=0;
    for(int i=0;i<NAV_DISTANCE_W*NAV_DISTANCE_H;i++)if(pixels[i]) {
        int head=0,tail=1,x0=i%NAV_DISTANCE_W,x1=x0,y0=i/NAV_DISTANCE_W,y1=y0;
        queue[0]=(uint16_t)i;pixels[i]=0;
        while(head<tail) {
            int pos=queue[head++],x=pos%NAV_DISTANCE_W,y=pos/NAV_DISTANCE_W;
            if(x<x0)x0=x;
            if(x>x1)x1=x;
            if(y<y0)y0=y;
            if(y>y1)y1=y;
            for(int dy=-1;dy<=1;dy++)for(int dx=-1;dx<=1;dx++) {
                int xx=x+dx,yy=y+dy;
                if(xx<0||xx>=NAV_DISTANCE_W||yy<0||yy>=NAV_DISTANCE_H)continue;
                int next=yy*NAV_DISTANCE_W+xx;
                if(pixels[next]){pixels[next]=0;queue[tail++]=(uint16_t)next;}
            }
        }
        if(tail<8)continue;
        if(count==128)return -1;
        struct nav_box box={x0,y0,x1-x0+1,y1-y0+1};int at=count++;
        while(at>0&&boxes[at-1].x>box.x){boxes[at]=boxes[at-1];at--;}
        boxes[at]=box;
    }
    return count;
}
int adas_nav_parse_rgb565(const uint8_t *im,struct adas_nav_observation *out) {
    if(!out)return -1;
    memset(out,0,sizeof(*out));
    if(!im)return -1;
    struct nav_box arrow;int area=nav_bounds(im,(struct nav_box){23,15,83,85},&arrow);
    if(area<180||arrow.w<14||arrow.h<34)return 0;
    float mask[NAV_GLYPH_W*NAV_GLYPH_H],best=0,second=0;
    enum adas_nav_direction direction=NAV_UNKNOWN;
    nav_normalize(im,arrow,mask);
    for(int i=0;i<3;i++) {
        const char *label=nav_arrow_labels[i];
        if(!label)continue;
        int right=!strcmp(label,"right"),left_arrow=!strcmp(label,"left"),straight=!strcmp(label,"straight");
        if(!right&&!left_arrow&&!straight)continue;
        float score=nav_similarity(mask,nav_arrow_templates[i],0);
        if(score>best) {
            second=best;best=score;direction=straight?NAV_STRAIGHT:left_arrow?NAV_LEFT:NAV_RIGHT;
        } else if(score>second)second=score;
    }
    out->direction_confidence=best;
    if(best<.80f||best-second<.10f)return 0;
    /* Connected components and unit rules mirror the calibrated Python parser,
     * including decimal kilometres (1.2 km is 1200 m, never a near turn). */
    struct nav_box boxes[128];int size=nav_components(im,boxes);
    if(size<=0)return 0;
    int digits=0,distance=0,end=0,first=-1;float digit_confidence=1;
    for(int i=0;i<size;i++)if(boxes[i].h>=38) {
        struct nav_box box=boxes[i];
        if(box.w<5||box.w>55||digits>=4)return 0;
        box.x+=NAV_DISTANCE_X;box.y+=NAV_DISTANCE_Y;
        float confidence;int digit=nav_digit(im,box,&confidence);
        if(digit<0)return 0;
        if(first<0)first=boxes[i].x;
        end=boxes[i].x+boxes[i].w;
        distance=distance*10+digit;digits++;
        digit_confidence=fminf(digit_confidence,confidence);
    }
    if(!digits)return 0;
    int km=0,metre=0;
    for(int i=0;i<size;i++) {
        struct nav_box b=boxes[i];
        if(b.x<end||b.x>=end+59||b.h>=38)continue;
        /* RGB565 thresholding can split the thin stem of 'k' from its lower
         * strokes. Recover that unit's bbox before distinguishing km from m;
         * otherwise a genuine "1 km" becomes a dangerous near "1 m". */
        int top=b.y,bottom=b.y+b.h;
        for(int j=0;j<size;j++) {
            struct nav_box c=boxes[j];
            if(c.x<end||c.x>=end+59||c.h>=38||c.x>=b.x+b.w||b.x>=c.x+c.w||
               c.y>bottom+3||top>c.y+c.h+3)continue;
            if(c.y<top)top=c.y;
            if(c.y+c.h>bottom)bottom=c.y+c.h;
        }
        km|=bottom-top>=26&&top<40;
        metre|=b.h>=12&&b.h<=26&&b.w>=12;
    }
    if(km) {
        int dots=0,after=0;
        for(int i=0;i<size;i++)if(boxes[i].w<=15&&boxes[i].h<=16&&boxes[i].y>43&&boxes[i].x>first&&boxes[i].x<end) {
            dots++;
            for(int j=0;j<size;j++)if(boxes[j].h>=38&&boxes[j].x>boxes[i].x)after++;
        }
        if(dots>1||after>3)return 0;
        int divisor=1;for(int i=0;i<after;i++)divisor*=10;
        distance=distance*1000/divisor;
    } else if(!metre)return 0;
    out->valid=1;out->direction=direction;out->distance_m=distance;
    out->distance_confidence=digit_confidence;out->confidence=fminf(best,digit_confidence);
    out->near_turn=distance<=120&&direction!=NAV_STRAIGHT;
    return 1;
}
int adas_nav_slot_init(struct adas_nav_slot *slot) {
    memset(slot,0,sizeof(*slot));slot->original=malloc(IMAGE_BYTES);return slot->original?0:-1;
}
void adas_nav_slot_close(struct adas_nav_slot *slot) {
    free(slot->original);memset(slot,0,sizeof(*slot));
}
int adas_nav_capture(struct adas_nav_slot *slot,uint8_t *road,struct features *f) {
    if(!slot||!slot->original||!road||!f)return -1;
    memcpy(slot->original,road,IMAGE_BYTES);
    if(adas_nav_parse_rgb565(slot->original,&slot->observation)<0)return -1;
    /* Read retained source, write the owned preprocessing source. Trailer stays
     * validated original metadata; it is never parsed again after this resize. */
    int columns[ADAS_W];
    for(int x=0;x<ADAS_W;x++)columns[x]=ADAS_NAV_ROAD_X+(2*x+1)*ADAS_NAV_ROAD_W/(2*ADAS_W);
    for(int y=0;y<ADAS_H;y++) {
        int sy=ADAS_NAV_ROAD_Y+(2*y+1)*ADAS_NAV_ROAD_H/(2*ADAS_H);
        for(int x=0;x<ADAS_W;x++) {
            size_t dst=2u*(y*ADAS_W+x),src=2u*(sy*ADAS_W+columns[x]);
            road[dst]=slot->original[src];road[dst+1]=slot->original[src+1];
        }
    }
    memset(f->candidates,0,sizeof(f->candidates));memset(f->road,0,sizeof(f->road));
    f->source_kind&=~(ADAS_SOURCE_QUAD|ADAS_SOURCE_QUAD_QUARTER);
    f->layout=0x20100;
    for(int y=0;y<ADAS_H;y+=4)for(int x=0;x<ADAS_W;x+=4) {
        unsigned v=nav_pixel(road,x,y),r=(v>>11)*8,g=((v>>5)&63)*4,b=(v&31)*8;
        unsigned gray=(77*r+150*g+29*b)>>8;
        f->road[0]++;f->road[1]+=gray;f->road[2]+=gray<40;
    }
    return 0;
}
void adas_nav_compose(struct adas_nav_slot *slot,const uint8_t *road) {
    if(!slot||!slot->original||!road)return;
    int columns[ADAS_NAV_ROAD_W];
    for(int x=0;x<ADAS_NAV_ROAD_W;x++)columns[x]=(2*x+1)*ADAS_W/(2*ADAS_NAV_ROAD_W);
    for(int y=0;y<ADAS_NAV_ROAD_H;y++) {
        int sy=(2*y+1)*ADAS_H/(2*ADAS_NAV_ROAD_H);
        for(int x=0;x<ADAS_NAV_ROAD_W;x++) {
            size_t dst=2u*((y+ADAS_NAV_ROAD_Y)*ADAS_W+x+ADAS_NAV_ROAD_X),src=2u*(sy*ADAS_W+columns[x]);
            slot->original[dst]=road[src];slot->original[dst+1]=road[src+1];
        }
    }
}
void adas_nav_update(struct adas_nav_state *s,const struct adas_nav_observation *o,
    uint32_t source,double arrived,int reset) {
    if(reset||!isfinite(arrived)||(s->have_frame&&
       ((uint32_t)(source-s->source)>15||source==s->source||arrived<=s->arrived||arrived-s->arrived>.5))) {
        unsigned char *mask=s->signal_mask;int *queue=s->signal_queue;
        memset(s,0,sizeof(*s));s->signal_mask=mask;s->signal_queue=queue;
    }
    s->have_frame=1;s->source=source;s->arrived=arrived;
    if(!o||!o->valid) { s->route=(struct adas_nav_observation){0};s->pending=NAV_UNKNOWN;s->hits=0;return; }
    int change=abs(o->distance_m-s->pending_distance);
    int tolerance=s->pending_distance/8;if(tolerance<30)tolerance=30;
    if(s->pending!=o->direction||change>tolerance) {
        s->pending=o->direction;s->hits=1;s->route=(struct adas_nav_observation){0};
    } else if(s->hits<3)s->hits++;
    s->pending_distance=o->distance_m;
    /* Never hold the previous direction/distance through an invalid or
     * unconfirmed observation, especially a far/near transition. */
    if(s->hits>=3) {
        s->route=*o;
        enum adas_nav_direction current=adas_nav_current_direction(&s->route);
        if(current==NAV_LEFT||current==NAV_RIGHT) {
            s->ar_direction=current;s->ar_turn_until=arrived+2.2;
        } else if(arrived>=s->ar_turn_until)s->ar_direction=current;
    }
}
enum adas_nav_direction adas_nav_current_direction(const struct adas_nav_observation *route) {
    if(!route||!route->valid)return NAV_UNKNOWN;
    if((route->direction==NAV_LEFT||route->direction==NAV_RIGHT)&&
       (route->distance_m<0||route->distance_m>120))return NAV_STRAIGHT;
    return route->direction;
}
int adas_nav_current_control(const struct adas_nav_observation *route) {
    struct adas_nav_observation current={0};
    if(route)current=*route;
    current.direction=adas_nav_current_direction(route);
    return adas_nav_control_route(&current);
}

static int nav_excess(const uint8_t *im,int x,int y,int color) {
    unsigned p=nav_pixel(im,x,y);
    int r=(int)(p>>11)*8,g=(int)((p>>5)&63)*4,b=(int)(p&31)*8;
    return color==LIGHT_RED?r-(g>b?g:b):g>100&&g*10>=b*9?g-r:0;
}
struct nav_lamp { int x,y,w,h,color; enum adas_nav_direction direction; float score; };
static int nav_patch_value(const uint8_t *im,int cx,int cy,int radius) {
    if(cx-radius<0||cy-radius<0||cx+radius>=ADAS_W||cy+radius>=ADAS_H)return -1;
    int sum=0,n=0;
    for(int y=cy-radius;y<=cy+radius;y++)for(int x=cx-radius;x<=cx+radius;x++) {
        unsigned p=nav_pixel(im,x,y);int r=(int)(p>>11)*8,g=(int)((p>>5)&63)*4,b=(int)(p&31)*8;
        int value=r>g?r:g;if(b>value)value=b;sum+=value;n++;
    }
    return n?sum/n:-1;
}
static int nav_vertical_housing(const uint8_t *im,const struct nav_lamp *a) {
    int diameter=a->w>a->h?a->w:a->h,radius=(int)(diameter*.22f);if(radius<1)radius=1;
    for(int sign=-1;sign<=1;sign+=2) {
        int hits=0;
        for(int step=0;step<2;step++) {
            int cx=a->x+a->w/2,cy=a->y+a->h/2+(int)roundf(sign*(step?2.1f:1.2f)*diameter);
            int v=nav_patch_value(im,cx,cy,radius),l=nav_patch_value(im,cx-diameter,cy,radius),r=nav_patch_value(im,cx+diameter,cy,radius);
            if(v>=0&&v<=165&&l-v>=22&&r-v>=22)hits++;
        }
        if(hits==2)return 1;
    }
    return 0;
}
static enum adas_nav_direction nav_arrow(const uint8_t *im,struct nav_lamp *lamp) {
    int peak=0;
    for(int y=lamp->y;y<lamp->y+lamp->h;y++)for(int x=lamp->x;x<lamp->x+lamp->w;x++) {
        int value=nav_excess(im,x,y,lamp->color);if(value>peak)peak=value;
    }
    int threshold=(int)(peak*.65f);if(threshold<40)threshold=40;
    int x0=ADAS_W,y0=ADAS_H,x1=-1,y1=-1;
    for(int y=lamp->y;y<lamp->y+lamp->h;y++)for(int x=lamp->x;x<lamp->x+lamp->w;x++)if(nav_excess(im,x,y,lamp->color)>threshold) {
        if(x<x0)x0=x;
        if(x>x1)x1=x;
        if(y<y0)y0=y;
        if(y>y1)y1=y;
    }
    if(x1<x0||y1<y0)return NAV_UNKNOWN;
    unsigned short rows[16]={0};
    for(int y=0;y<16;y++)for(int x=0;x<16;x++)
        if(nav_excess(im,x0+x*(x1-x0+1)/16,y0+y*(y1-y0+1)/16,lamp->color)>threshold)rows[y]|=(unsigned short)(1u<<x);
    float scores[4]={0};
    for(size_t i=0;i<sizeof(nav_signal_templates)/sizeof(nav_signal_templates[0]);i++) {
        unsigned intersection=0,total=0;
        for(int y=0;y<16;y++)for(int x=0;x<16;x++) {
            unsigned a=(rows[y]>>x)&1,b=(nav_signal_templates[i].rows[y]>>x)&1;
            intersection+=a&b;total+=a|b;
        }
        float score=total?(float)intersection/total:0;
        enum adas_nav_direction d=nav_signal_templates[i].direction;
        if(score>scores[d])scores[d]=score;
    }
    enum adas_nav_direction best=NAV_UNKNOWN;float first=0,second=0;
    for(int d=NAV_LEFT;d<=NAV_STRAIGHT;d++)if(scores[d]>first) {
        second=first;first=scores[d];best=(enum adas_nav_direction)d;
    } else if(scores[d]>second)second=scores[d];
    lamp->score=first;
    return first>=.72f&&first-second>=.025f?best:NAV_UNKNOWN;
}
static int nav_lamp_on_vehicle(const struct detection *objects,int count,int x,int y) {
    if(!objects)return 0;
    for(int i=0;i<count;i++)if(objects[i].cls>=0&&objects[i].cls<5&&
        x>=objects[i].x0&&x<=objects[i].x1&&y>=objects[i].y0&&y<=objects[i].y1)return 1;
    return 0;
}
void adas_nav_pixel_signal_with_objects(struct adas_nav_state *s,const uint8_t *im,
    const struct detection *objects,int object_count,double stamp,struct adas_signal *out) {
    if(!s||!im||!out)return;
    *out=(struct adas_signal){0};
    enum { SW=960,SH=350,N=SW*SH };
    if(!s->signal_mask)s->signal_mask=malloc(N);
    if(!s->signal_queue)s->signal_queue=malloc(N*sizeof(int));
    unsigned char *mask=s->signal_mask;int *queue=s->signal_queue;
    if(!mask||!queue)return;
    memset(mask,0,N);
    if(!isfinite(stamp)||stamp<=s->signal_frame||stamp-s->signal_frame>1)s->signal_have=0;
    s->signal_frame=stamp;
    struct nav_lamp lamps[64];int count=0;
    for(int y=0;y<SH;y++)for(int x=0;x<SW;x++) {
        int px=x*2,py=y*2;
        if(px<ADAS_W*.15f||px>ADAS_W*.90f)continue;
        int red=nav_excess(im,px,py,LIGHT_RED),green=nav_excess(im,px,py,LIGHT_GREEN);
        if(red>40)mask[y*SW+x]=1;else if(green>40)mask[y*SW+x]=2;
    }
    for(int offset=0;offset<N;offset++)if(mask[offset]) {
        int color=mask[offset],head=0,tail=1,x0=offset%SW,x1=x0,y0=offset/SW,y1=y0;
        queue[0]=offset;mask[offset]=0;
        while(head<tail) {
            int at=queue[head++],x=at%SW,y=at/SW;
            if(x<x0)x0=x;
            if(x>x1)x1=x;
            if(y<y0)y0=y;
            if(y>y1)y1=y;
            for(int dy=-1;dy<=1;dy++)for(int dx=-1;dx<=1;dx++) {
                int nx=x+dx,ny=y+dy;
                if(nx<0||nx>=SW||ny<0||ny>=SH)continue;
                int next=ny*SW+nx;
                if(mask[next]==color){mask[next]=0;queue[tail++]=next;}
            }
        }
        int w=(x1-x0+1)*2,h=(y1-y0+1)*2;
#ifdef NAV_SIGNAL_DEBUG
        if(x0*2>700&&x0*2<1000&&y0*2>300&&y0*2<550)fprintf(stderr,"BLOB %.2f %d,%d %dx%d count=%d fill=%.3f\n",stamp,x0*2,y0*2,w,h,tail,(float)tail*4/(w*h));
#endif
        if(tail<8||w<8||h<8||w>65||h>65||w>2*h||h>2*w||tail*4>w*h*.94f||tail*4<w*h*.18f)continue;
        int dark=0,samples=0;
        for(int y=y0*2-h/2;y<=y1*2+h/2;y+=2)for(int x=x0*2-w/2;x<=x1*2+w/2;x+=2) {
            if(x<0||x>=ADAS_W||y<0||y>=ADAS_H)continue;
            if(x>=x0*2&&x<=x1*2&&y>=y0*2&&y<=y1*2)continue;
            unsigned p=nav_pixel(im,x,y);samples++;
            if((p>>11)*8<130&&((p>>5)&63)*4<130&&(p&31)*8<130)dark++;
        }
#ifdef NAV_SIGNAL_DEBUG
        if(x0*2>700&&x0*2<1000&&y0*2>300&&y0*2<550)fprintf(stderr,"HOUSING %.2f dark=%d total=%d\n",stamp,dark,samples);
#endif
        if(!samples||dark<samples*.15f||count>=64)continue;
        struct nav_lamp lamp={.x=x0*2,.y=y0*2,.w=w,.h=h,.color=color==1?LIGHT_RED:LIGHT_GREEN};
        lamp.direction=nav_arrow(im,&lamp);lamps[count++]=lamp;
#ifdef NAV_SIGNAL_DEBUG
        fprintf(stderr,"LAMP %.3f xy=%d,%d wh=%d,%d color=%d dir=%d score=%.3f\n",stamp,lamp.x,lamp.y,lamp.w,lamp.h,lamp.color,lamp.direction,lamp.score);
#endif
    }
    enum adas_nav_direction direction=adas_nav_current_direction(&s->route);
    int selected=-1;
    if(s->signal_have&&nav_lamp_on_vehicle(objects,object_count,s->signal_x+10,s->signal_y+10)) {
        s->signal_have=0;s->signal_hits=0;
    }
    /* Once shape + adjacent-head context has acquired an applicable lamp,
     * follow fresh color pixels inside that small head window. This bridges
     * RGB565 silhouette/housing jitter, never a missing image or a color-free
     * hold. The adjacent countdown lies outside the tracking window. */
    struct nav_lamp tracked={0};int have_tracked=0;
    if(s->signal_have&&s->signal_direction==direction&&stamp-s->signal_seen<=.35) {
        int counts[2]={0},sums_x[2]={0},sums_y[2]={0};
        int mins_x[2]={ADAS_W,ADAS_W},mins_y[2]={ADAS_H,ADAS_H};
        for(int y=s->signal_y-8;y<=s->signal_y+28;y++)for(int x=s->signal_x-8;x<=s->signal_x+28;x++) {
            if(x<0||x>=ADAS_W||y<0||y>=700)continue;
            for(int k=0;k<2;k++)if(nav_excess(im,x,y,k?LIGHT_GREEN:LIGHT_RED)>65) {
                counts[k]++;sums_x[k]+=x;sums_y[k]+=y;
                if(x<mins_x[k])mins_x[k]=x;
                if(y<mins_y[k])mins_y[k]=y;
            }
        }
        int k=counts[1]>counts[0]?1:0;
        if(counts[k]>=12&&counts[1-k]<counts[k]/3) {
            tracked=(struct nav_lamp){.x=mins_x[k],.y=mins_y[k],.w=20,.h=20,
                .color=k?LIGHT_GREEN:LIGHT_RED,.direction=direction,.score=.8f};
            /* Bound displacement so a different object cannot take the lock. */
            if(abs(sums_x[k]/counts[k]-(s->signal_x+10))<20&&
               abs(sums_y[k]/counts[k]-(s->signal_y+10))<20)have_tracked=1;
        }
    }
    for(int i=0;i<count;i++) {
        struct nav_lamp *a=&lamps[i];
        if(nav_lamp_on_vehicle(objects,object_count,a->x+a->w/2,a->y+a->h/2))continue;
        if(direction==NAV_UNKNOWN||a->direction!=direction)continue;
        int context=nav_vertical_housing(im,a);
        for(int j=0;j<count;j++)if(j!=i) {
            struct nav_lamp *b=&lamps[j];
            int gap=abs(a->x-b->x),height=a->h>b->h?a->h:b->h;
            if(gap>2*(a->w>b->w?a->w:b->w)&&gap<ADAS_W*.3f&&abs(a->y-b->y)<3*height&&b->w>a->w*.5f&&b->w<a->w*2)context=1;
        }
        if(context&&(selected<0||a->score>lamps[selected].score))selected=i;
    }
    if(selected>=0||have_tracked) {
        struct nav_lamp *a=have_tracked?&tracked:&lamps[selected];
        int same=s->signal_have&&s->signal_direction==direction&&s->signal_color==a->color&&
            stamp-s->signal_seen<=.6&&abs(a->x-s->signal_x)<44&&abs(a->y-s->signal_y)<64;
        if(!same){s->signal_first=stamp;s->signal_hits=0;}
        s->signal_have=1;s->signal_direction=direction;s->signal_color=a->color;
        s->signal_x=a->x;s->signal_y=a->y;s->signal_seen=stamp;s->signal_hits++;
    } else if(!s->signal_have||stamp-s->signal_seen>.35||s->signal_direction!=direction)s->signal_have=0;
    if(s->signal_have&&s->signal_hits>=3&&stamp-s->signal_first>=.3) {
        out->candidate=(enum adas_light)s->signal_color;out->source=SIGNAL_ROI;
        out->stable=1;out->confidence=.8f;
    }
}
void adas_nav_pixel_signal(struct adas_nav_state *s,const uint8_t *im,double stamp,struct adas_signal *out) {
    adas_nav_pixel_signal_with_objects(s,im,NULL,0,stamp,out);
}
void adas_nav_close(struct adas_nav_state *s) {
    free(s->signal_mask);free(s->signal_queue);memset(s,0,sizeof(*s));
}
const char *adas_nav_direction_name(enum adas_nav_direction direction) {
    switch(direction){case NAV_LEFT:return "left";case NAV_RIGHT:return "right";case NAV_STRAIGHT:return "straight";default:return "unknown";}
}
int adas_nav_control_route(const struct adas_nav_observation *route) {
    if(!route||!route->valid)return 2;
    switch(route->direction){case NAV_LEFT:return -1;case NAV_RIGHT:return 1;case NAV_STRAIGHT:return 0;default:return 2;}
}
static void nav_observation_json(FILE *file,const struct adas_nav_observation *o) {
    fprintf(file,"{\"valid\":%s,\"direction\":\"%s\",\"distance_m\":",o->valid?"true":"false",adas_nav_direction_name(o->direction));
    if(o->valid)fprintf(file,"%d",o->distance_m);else fputs("null",file);
    fprintf(file,",\"confidence\":%.5f,\"direction_confidence\":%.5f,\"distance_confidence\":%.5f,\"near_turn\":%s,\"stage\":\"%s\"}",
        o->confidence,o->direction_confidence,o->distance_confidence,o->near_turn?"true":"false",
        !o->valid?"unknown":o->distance_m<=120?"near":o->distance_m<=500?"prepare":"far");
}
void adas_nav_write_json(FILE *file,const struct adas_nav_observation *pixel,const struct adas_nav_observation *route) {
    fputs("\"navigation\":{\"source\":\"native_rgb565_atlas\",\"pixel\":",file);nav_observation_json(file,pixel);
    fputs(",\"route\":",file);nav_observation_json(file,route);
    fprintf(file,",\"road_roi\":[%d,%d,%d,%d],\"arrow_roi\":[23,15,83,85],\"distance_roi\":[109,20,190,74],\"road_coordinates\":\"normalized_1920x1080\"},",
        ADAS_NAV_ROAD_X,ADAS_NAV_ROAD_Y,ADAS_NAV_ROAD_W,ADAS_NAV_ROAD_H);
}
