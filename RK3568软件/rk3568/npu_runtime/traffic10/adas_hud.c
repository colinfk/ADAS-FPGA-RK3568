#include "adas_hud.h"
#include "hud_font.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#ifndef HUD_UI_V2
#define HUD_UI_V2 1
#endif
#ifndef HUD_CRUISE_STAGE
#define HUD_CRUISE_STAGE 8
#endif
#if HUD_UI_V2 && !defined(HUD_TEST_RAW_TEXT)
#include "cruise_font.h"
#endif
/* Mode selection is UI-only; the default keeps the existing runtime behavior. */
static enum hud_mode g_hud_mode=HUD_MODE_INTERSECTION;
static int g_hud_mode_initialized;
static int g_hud_mode_explicit;
static unsigned g_hud_mode_generation;
static int intersection_app_render;
static int pixel_light_hud;
void adas_hud_set_pixel_light(int on) { pixel_light_hud=on?1:0; }
void adas_hud_set_mode(enum hud_mode mode) {
    g_hud_mode=mode>=HUD_MODE_CRUISE&&mode<=HUD_MODE_SURROUND?mode:HUD_MODE_INTERSECTION;
    g_hud_mode_initialized=1;
    g_hud_mode_explicit=1;
    g_hud_mode_generation++;
}
#if HUD_UI_V2 && !defined(HUD_TEST_RAW_TEXT)
enum { HUD_PANEL_DARK=1, HUD_PANEL_SIGNAL=2, HUD_PANEL_STRAIGHT=4 };
struct hud_mode_config { const char *key,*title; unsigned panels; uint32_t suppressed; };
static const struct hud_mode_config hud_modes[]={
    [HUD_MODE_CRUISE]={"cruise","巡航模式",HUD_PANEL_DARK,EVENT_RED_MOVING|EVENT_RED_STOP|EVENT_PEDESTRIAN},
    [HUD_MODE_INTERSECTION]={"intersection","路口模式",HUD_PANEL_DARK|HUD_PANEL_SIGNAL|HUD_PANEL_STRAIGHT,0},
    [HUD_MODE_NAV]={"nav","导航模式",HUD_PANEL_DARK|HUD_PANEL_SIGNAL,0},
    [HUD_MODE_SURROUND]={"surround","环视模式",HUD_PANEL_DARK|HUD_PANEL_STRAIGHT,EVENT_RED_MOVING|EVENT_RED_STOP}
};
static int cruise_active(void) {
    if(!g_hud_mode_initialized) {
        const char *mode=getenv("ADAS_HUD_MODE");
        for(unsigned i=0;mode&&i<sizeof(hud_modes)/sizeof(hud_modes[0]);i++)
            if(!strcmp(mode,hud_modes[i].key)){g_hud_mode=(enum hud_mode)i;g_hud_mode_explicit=1;break;}
        g_hud_mode_initialized=1;
    }
    return g_hud_mode_explicit;
}
static int cruise_debug;
static const struct intersection_view *intersection_live;
#endif
#define CYAN 0x54e4e5u
#define GREEN 0x5ce5b0u
#define AMBER 0xffc45cu
#define RED 0xff454fu
#define WHITE 0xf1f6fau
#define MUTED 0x9db3c3u
#define PANEL 0x101e2du
#define TOPBAR 0x0b1220u /* competition premium top chrome */
#define CAPSULE 0x152238u
static unsigned color565(unsigned c) { return ((c>>19)&31)<<11|((c>>10)&63)<<5|((c>>3)&31); }
static void blend(uint8_t *image,int x,int y,unsigned color,unsigned alpha) {
    if(x<0||x>=ADAS_W||y<0||y>=ADAS_H||!alpha) return;
    uint8_t *p=image+(y*ADAS_W+x)*2;
    if(alpha>=255) { unsigned c=color565(color); p[0]=c&255; p[1]=c>>8; return; }
    unsigned value=p[0]|(unsigned)p[1]<<8;
    unsigned fg=color565(color),a=alpha+1,inv=256-a;
    unsigned r=(((value>>11)&31)*inv+((fg>>11)&31)*a)>>8;
    unsigned g=(((value>>5)&63)*inv+((fg>>5)&63)*a)>>8;
    unsigned b=((value&31)*inv+(fg&31)*a)>>8;
    value=(r<<11)|(g<<5)|b;
    p[0]=value&255; p[1]=value>>8;
}
static void rect(uint8_t *im,int x,int y,int w,int h,unsigned c,unsigned alpha) {
    int x0=x<0?0:x,y0=y<0?0:y,x1=x+w>ADAS_W?ADAS_W:x+w,y1=y+h>ADAS_H?ADAS_H:y+h;
    for(int row=y0;row<y1;row++) for(int col=x0;col<x1;col++) blend(im,col,row,c,alpha);
}
static void rounded(uint8_t *im,int x,int y,int w,int h,int radius,unsigned c,unsigned alpha) {
    for(int row=0;row<h;row++) for(int col=0;col<w;col++) {
        int dx=col<radius?radius-col:col>=w-radius?col-(w-radius-1):0;
        int dy=row<radius?radius-row:row>=h-radius?row-(h-radius-1):0;
        if(dx*dx+dy*dy<=radius*radius) blend(im,x+col,y+row,c,alpha);
    }
}
static void line(uint8_t *im,int x0,int y0,int x1,int y1,int thick,unsigned c,unsigned alpha) {
    int dx=abs(x1-x0),sx=x0<x1?1:-1,dy=-abs(y1-y0),sy=y0<y1?1:-1,err=dx+dy;
    for(;;) {
        rect(im,x0-thick/2,y0-thick/2,thick,thick,c,alpha);
        if(x0==x1&&y0==y1) break;
        int twice=2*err;
        if(twice>=dy) { err+=dy; x0+=sx; } if(twice<=dx) { err+=dx; y0+=sy; }
    }
}
static uint32_t next_utf8(const unsigned char **str) {
    const unsigned char *p=*str; uint32_t c=*p++;
    if(c>=0xe0&&c<0xf0&&p[0]&&p[1]) { c=((c&15)<<12)|((p[0]&63)<<6)|(p[1]&63); p+=2; }
    else if(c>=0xc0&&c<0xe0&&p[0]) { c=((c&31)<<6)|(p[0]&63); p++; }
    else if(c>=128) c='?';
    *str=p; return c;
}
static const struct hud_glyph *glyph(uint32_t code) {
    size_t lo=0,hi=sizeof(hud_glyphs)/sizeof(hud_glyphs[0]);
    while(lo<hi) { size_t m=(lo+hi)/2; if(hud_glyphs[m].code<code) lo=m+1; else hi=m; }
    if(lo<sizeof(hud_glyphs)/sizeof(hud_glyphs[0])&&hud_glyphs[lo].code==code) return &hud_glyphs[lo];
    return &hud_glyphs['?'-32];
}
/* Translate only rendered labels; JSON and test-facing diagnostic tokens retain their ABI. */
#ifndef HUD_TEST_RAW_TEXT
static int ascii_word(unsigned char c) {
    return (c>='A'&&c<='Z')||(c>='a'&&c<='z')||(c>='0'&&c<='9')||c=='_';
}
#endif
static void localize_hud(const char *input,char *output,size_t size) {
#ifdef HUD_TEST_RAW_TEXT
    /* Test-only raw rendering proves that localization did not move the original layout. */
    snprintf(output,size,"%s",input);return;
#else
    static const struct { const char *from,*to; size_t length; } labels[]={
        {"EVT","预警",sizeof("EVT")-1},
        {"SANDBOX","沙盒",sizeof("SANDBOX")-1},
        {"CHECK INPUT / DRIVER ATTENTION","数据超龄 暂停辅助提示",sizeof("CHECK INPUT / DRIVER ATTENTION")-1},
        {"PEDESTRIAN AHEAD","前方行人",sizeof("PEDESTRIAN AHEAD")-1},
        {"RED LIGHT - KEEP STOPPED","红灯亮起 保持停车",sizeof("RED LIGHT - KEEP STOPPED")-1},
        {"KEEP CENTER / WATCH EDGE","保持居中 留意边界",sizeof("KEEP CENTER / WATCH EDGE")-1},
        {"SLOW / WATCH ROAD USERS","减速 留意行人与车辆",sizeof("SLOW / WATCH ROAD USERS")-1},
        {"LANE CHANGE RESTRICTED","变道条件受限",sizeof("LANE CHANGE RESTRICTED")-1},
        {"KEEP WATCHING THE ROAD","持续观察道路",sizeof("KEEP WATCHING THE ROAD")-1},
        {"INCREASE FOLLOWING GAP","增大跟车间距",sizeof("INCREASE FOLLOWING GAP")-1},
        {"LANE POSITION UNKNOWN","车道位置尚未确认",sizeof("LANE POSITION UNKNOWN")-1},
        {"VERIFY TRAFFIC SIGNAL","确认交通信号",sizeof("VERIFY TRAFFIC SIGNAL")-1},
        {"SLOW / LOW VISIBILITY","能见度低 请减速",sizeof("SLOW / LOW VISIBILITY")-1},
        {"YELLOW / WATCH SIGNAL","黄灯亮起 留意信号",sizeof("YELLOW / WATCH SIGNAL")-1},
        {"SEARCH RED CANDIDATE","搜索到红灯候选",sizeof("SEARCH RED CANDIDATE")-1},
        {"VERIFY LANE POSITION","确认车道位置",sizeof("VERIFY LANE POSITION")-1},
        {"SOLID LINE CROSSING","按键模拟强变 · 实线禁越报警",sizeof("SOLID LINE CROSSING")-1},
        {"SIMULATED OVERSPEED","模拟车速超限",sizeof("SIMULATED OVERSPEED")-1},
        {"OBJECT APPROACHING","前方目标正在逼近",sizeof("OBJECT APPROACHING")-1},
        {"CROSSING CANDIDATE","斑马线候选",sizeof("CROSSING CANDIDATE")-1},
        {"JUNCTION CANDIDATE","路口候选",sizeof("JUNCTION CANDIDATE")-1},
        {"AVOID LANE CHANGE","暂缓变道",sizeof("AVOID LANE CHANGE")-1},
        {"DASHED_CHECK_REAR","虚线·留意侧后方",sizeof("DASHED_CHECK_REAR")-1},
        {"RED LIGHT - STOP","红灯期间模拟行驶 · 立即停车",sizeof("RED LIGHT - STOP")-1},
        {"OBJECT TOO CLOSE","前方目标距离过近",sizeof("OBJECT TOO CLOSE")-1},
        {"CUT IN CANDIDATE","邻车切入候选",sizeof("CUT IN CANDIDATE")-1},
        {"INVALID / NO GEO","无效·未定位",sizeof("INVALID / NO GEO")-1},
        {"UNKNOWN / NO GEO","未知·未定位",sizeof("UNKNOWN / NO GEO")-1},
        {"FRONT VIEW / ADJ","前视邻道",sizeof("FRONT VIEW / ADJ")-1},
        {"DASHED / NO GEO","虚线·未定位",sizeof("DASHED / NO GEO")-1},
        {"PREPARE TO STOP","准备停车",sizeof("PREPARE TO STOP")-1},
        {"LOW VISIBILITY","能见度偏低",sizeof("LOW VISIBILITY")-1},
        {"SOLID / NO GEO","实线·未定位",sizeof("SOLID / NO GEO")-1},
        {"KEEP OBSERVING","保持观察",sizeof("KEEP OBSERVING")-1},
        {"WATCH CROSSING","留意斑马线",sizeof("WATCH CROSSING")-1},
        {"CPU_GUIDED_ROI","CPU沿线区域",sizeof("CPU_GUIDED_ROI")-1},
        {"NONE / NO GEO","无标线·未定位",sizeof("NONE / NO GEO")-1},
        {"DENSE TRAFFIC","车流密集",sizeof("DENSE TRAFFIC")-1},
        {"OCCUPIED_WAIT","邻道占用·等待",sizeof("OCCUPIED_WAIT")-1},
        {"URBAN CONFIG","城区配置",sizeof("URBAN CONFIG")-1},
        {"AUTO CONTEXT","自动场景",sizeof("AUTO CONTEXT")-1},
        {"LANE MONITOR","车道监测",sizeof("LANE MONITOR")-1},
        {"KEEP STOPPED","保持停车",sizeof("KEEP STOPPED")-1},
        {"FRAME DELAY","画面帧超龄 仅供参考",sizeof("FRAME DELAY")-1},
        {"SEARCH LINK","灯控:搜索联动",sizeof("SEARCH LINK")-1},
        {"HWY CONFIG","高速配置",sizeof("HWY CONFIG")-1},
        {"SOLID_HOLD","实线·禁变道",sizeof("SOLID_HOLD")-1},
        {"LANE EDGE","靠近车道边界",sizeof("LANE EDGE")-1},
        {"ADAS DEMO","前视巡航",sizeof("ADAS DEMO")-1},
        {"TL SEARCH","搜索灯态",sizeof("TL SEARCH")-1},
        {"VRU WATCH","留意行人与骑行者",sizeof("VRU WATCH")-1},
        {"WAIT SYNC","等待同步",sizeof("WAIT SYNC")-1},
        {"UNCERTAIN","未确认",sizeof("UNCERTAIN")-1},
        {"ADAS SIM","按键模拟",sizeof("ADAS SIM")-1},
        {"TL WATCH","灯态待确认",sizeof("TL WATCH")-1},
        {"RED+MOVE","红灯行驶",sizeof("RED+MOVE")-1},
        {"RED STOP","红灯停车",sizeof("RED STOP")-1},
        {"FPGA_ROI","FPGA区域",sizeof("FPGA_ROI")-1},
        {"ROI HITS","区域命中",sizeof("ROI HITS")-1},
        {"CROSSING","斑马线",sizeof("CROSSING")-1},
        {"OCCUPIED","有目标",sizeof("OCCUPIED")-1},
        {"ROI FSM","灯控:区域/按键",sizeof("ROI FSM")-1},
        {"NO LANE","车道待确认",sizeof("NO LANE")-1},
        {"INVALID","无效",sizeof("INVALID")-1},
        {"UNKNOWN","未知",sizeof("UNKNOWN")-1},
        {"CAUTION","注意",sizeof("CAUTION")-1},
        {"MONITOR","监测",sizeof("MONITOR")-1},
        {"OBJECTS","目标",sizeof("OBJECTS")-1},
        {"TL SIM","模拟灯态",sizeof("TL SIM")-1},
        {"TL ROI","区域灯态",sizeof("TL ROI")-1},
        {"NO OBJ","未见目标",sizeof("NO OBJ")-1},
        {"CUT IN","切入提示",sizeof("CUT IN")-1},
        {"DASHED","虚线",sizeof("DASHED")-1},
        {"YELLOW","黄灯",sizeof("YELLOW")-1},
        {"DANGER","危险",sizeof("DANGER")-1},
        {"SOLID","实线",sizeof("SOLID")-1},
        {"GREEN","绿灯",sizeof("GREEN")-1},
        {"FORCE","强制变道",sizeof("FORCE")-1},
        {"BLOCK","变道受限",sizeof("BLOCK")-1},
        {"SPEED","车速",sizeof("SPEED")-1},
        {"STALE","帧过期",sizeof("STALE")-1},
        {"RULES","规则",sizeof("RULES")-1},
        {"FRAME","帧",sizeof("FRAME")-1},
        {"VIDEO","视频",sizeof("VIDEO")-1},
        {"FUSED","套用",sizeof("FUSED")-1},
        {"ZEBRA","斑马线",sizeof("ZEBRA")-1},
        {"NONE","无标线",sizeof("NONE")-1},
        {"NEAR","目标过近",sizeof("NEAR")-1},
        {"EDGE","道边",sizeof("EDGE")-1},
        {"DARK","暗区",sizeof("DARK")-1},
        {"RULE","规则",sizeof("RULE")-1},
        {"HOLD","沿用",sizeof("HOLD")-1},
        {"ADAS","驾驶辅助",sizeof("ADAS")-1},
        {"SYNC","同步",sizeof("SYNC")-1},
        {"WAIT","等待",sizeof("WAIT")-1},
        {"HITS","命中",sizeof("HITS")-1},
        {"CAND","候选",sizeof("CAND")-1},
        {"RED","红灯",sizeof("RED")-1},
        {"GRN","绿灯",sizeof("GRN")-1},
        {"YEL","黄灯",sizeof("YEL")-1},
        {"PED","行人提示",sizeof("PED")-1},
        {"GAP","裕度",sizeof("GAP")-1},
        {"ROI","区域",sizeof("ROI")-1},
        {"LUX","低照",sizeof("LUX")-1},
        {"OK","正常",sizeof("OK")-1},
        {"MS","毫秒",sizeof("MS")-1},
        {"L","左",sizeof("L")-1},
        {"R","右",sizeof("R")-1},
        {"F","帧",sizeof("F")-1},
        {"Y","亮度",sizeof("Y")-1},
    };
    size_t used=0;
    const char *p=input;
    if(!size)return;
    while(*p) {
        const char *replacement=NULL;size_t consumed=0;
        if((*p>='A'&&*p<='Z')||*p=='k')for(size_t i=0;i<sizeof(labels)/sizeof(labels[0]);i++) {
            size_t n=labels[i].length;
            if((p==input||!ascii_word((unsigned char)p[-1]))&&
               !strncmp(p,labels[i].from,n)&&!ascii_word((unsigned char)p[n])) {
                /* Slash-separated keyboard keys must remain physical key names. */
                if(n==1&&((p>input&&p[-1]=='/')||p[1]=='/'))continue;
                replacement=labels[i].to;consumed=n;break;
            }
        }
        size_t n;
        if(replacement)n=strlen(replacement);
        else {replacement=p;consumed=1;unsigned char c=(unsigned char)*p;
            if(c>=0xe0&&c<0xf0&&p[1]&&p[2])consumed=3;
            else if(c>=0xc0&&c<0xe0&&p[1])consumed=2;
            n=consumed;
        }
        if(n>=size-used)break;
        memcpy(output+used,replacement,n);used+=n;p+=consumed;
    }
    output[used]=0;
#endif
}

static int translated_width(const char *str,float scale) {
    int width=0; const unsigned char *s=(const unsigned char *)str;
    while(*s) width+=(int)lroundf(glyph(next_utf8(&s))->advance*scale);
    return width;
}
static int text_width(const char *str,float scale) {
    char translated[1024];localize_hud(str,translated,sizeof(translated));return translated_width(translated,scale);
}
static void translated_text(uint8_t *im,int x,int y,const char *str,unsigned color,float scale) {
    const unsigned char *s=(const unsigned char *)str;
    while(*s) {
        const struct hud_glyph *g=glyph(next_utf8(&s));
        int h=(int)lroundf(HUD_FONT_H*scale),w=(int)lroundf(HUD_FONT_W*scale);
        for(int row=0;row<h;row++) for(int col=0;col<w;col++) {
            int a=(int)(row/scale),b=(int)(col/scale);
            if(a<HUD_FONT_H&&b<HUD_FONT_W) blend(im,x+col,y+row,color,g->alpha[a*HUD_FONT_W+b]);
        }
        x+=(int)lroundf(g->advance*scale);
    }
}
static void text(uint8_t *im,int x,int y,const char *str,unsigned color,float scale) {
    char translated[1024];localize_hud(str,translated,sizeof(translated));translated_text(im,x,y,translated,color,scale);
}
void adas_hud_return_button(uint8_t *im) {
    const char *enabled=getenv("ADAS_LAUNCHER_RETURN_UI");
    if(!enabled||strcmp(enabled,"1"))return;
    rounded(im,ADAS_HUD_RETURN_X,ADAS_HUD_RETURN_Y,ADAS_HUD_RETURN_W,ADAS_HUD_RETURN_H,12,PANEL,240);
    text(im,ADAS_HUD_RETURN_X+36,ADAS_HUD_RETURN_Y+4,"返回主页",WHITE,.8f);
    text(im,ADAS_HUD_RETURN_X+64,ADAS_HUD_RETURN_Y+37,"Esc",CYAN,.8f);
}
#ifndef HUD_TEST_RAW_TEXT
static void hud_turn_edges(uint8_t *im,const struct adas_status *s,unsigned color,unsigned opacity) {
    if(!opacity)return;
    float pulse=isfinite(s->phase_time)?.5f+.5f*sinf((float)(s->phase_time*12.56637061)):1.f;
    unsigned alpha=(unsigned)(90+150*pulse)*opacity/255;
    rect(im,0,0,ADAS_W,12,color,alpha);
    rect(im,0,ADAS_H-12,ADAS_W,12,color,alpha);
    rect(im,0,12,12,ADAS_H-24,color,alpha);
    rect(im,ADAS_W-12,12,12,ADAS_H-24,color,alpha);
}
static void centered(uint8_t *im,int center,int y,const char *str,unsigned color,float scale) {
    char translated[1024];localize_hud(str,translated,sizeof(translated));
    translated_text(im,center-translated_width(translated,scale)/2,y,translated,color,scale);
}
#endif
#ifdef HUD_TEST_RAW_TEXT
static void arrow(uint8_t *im,int x,int y,int direction,unsigned color,int on) {
    unsigned a=on?255:55;
    if(!direction) {
        line(im,x,y+48,x,y-30,9,color,a); line(im,x,y-30,x-24,y-4,9,color,a); line(im,x,y-30,x+24,y-4,9,color,a);
    } else {
        line(im,x,y+48,x,y+6,9,color,a); line(im,x,y+6,x+direction*48,y+6,9,color,a);
        line(im,x+direction*48,y+6,x+direction*24,y-18,9,color,a);
        line(im,x+direction*48,y+6,x+direction*24,y+30,9,color,a);
    }
}
#endif
static void corners(uint8_t *im,int x0,int y0,int x1,int y1,unsigned color,int width) {
    int len=(x1-x0)/4; if(len>28) len=28; if(len<4) len=4;
    for(int i=0;i<2;i++) for(int j=0;j<2;j++) {
        int x=i?x1:x0,y=j?y1:y0;
        line(im,x,y,x+(i?-len:len),y,width,color,255);
        line(im,x,y,x,y+(j?-len:len),width,color,255);
    }
}
static const char *class_cn(int cls) {
    static const char *names[]={"汽车","卡车","公交车","摩托车","自行车","行人"};
    return cls>=0&&cls<6?names[cls]:"目标";
}
void adas_hud_yolo_only(uint8_t *im,const struct detection *objects,int count,uint32_t frame_id) {
    rect(im,0,0,ADAS_W,64,PANEL,255);
    char title[128];
    snprintf(title,sizeof(title),"YOLO / RK3568     FRAME %u     OBJECTS %d",frame_id,count);
    text(im,28,16,title,WHITE,.8f);
    int label_x[MAX_DETECTIONS],label_y[MAX_DETECTIONS],label_w[MAX_DETECTIONS],labels=0;
    if(count>MAX_DETECTIONS)count=MAX_DETECTIONS;
    for(int i=0;i<count;i++) {
        const struct detection *d=&objects[i];
        if(!isfinite(d->x0)||!isfinite(d->x1)||!isfinite(d->y0)||!isfinite(d->y1)||
           !isfinite(d->score)||d->x1<=d->x0||d->y1<=d->y0)continue;
        int x=(int)fmaxf(0,fminf(ADAS_W-1,d->x0)),y=(int)fmaxf(0,fminf(ADAS_H-1,d->y0));
        int right=(int)fmaxf(0,fminf(ADAS_W-1,d->x1));
        int bottom=(int)fmaxf(0,fminf(ADAS_H-1,d->y1));
        corners(im,x,y,right,bottom,GREEN,3);
        char label[96];snprintf(label,sizeof(label),"%s %.0f%%",class_cn(d->cls),d->score*100);
        int width=text_width(label,.7f)+20;if(x+width>ADAS_W)x=ADAS_W-width;
        int anchor=y<66?66:y>ADAS_H-34?ADAS_H-34:y;
        for(int attempt=0;attempt<60;attempt++) {
            y=anchor+(attempt==0?0:(attempt%2?1:-1)*((attempt+1)/2)*36);
            if(y<66||y>ADAS_H-34)continue;
            int collision=0;
            for(int j=0;j<labels;j++)if(x<label_x[j]+label_w[j]+4&&x+width+4>label_x[j]&&
                y<label_y[j]+36&&y+36>label_y[j]){collision=1;break;}
            if(!collision)break;
        }
        if(y<66||y>ADAS_H-34)y=66;
        label_x[labels]=x;label_y[labels]=y;label_w[labels++]=width;
        rect(im,x,y, width,32,PANEL,255);text(im,x+10,y+2,label,WHITE,.7f);
    }
}
#ifdef HUD_TEST_RAW_TEXT
static void side_card(uint8_t *im,int side,enum adas_hint hint,int turn,double phase) {
    int x=side?1544:32;
    unsigned color=hint==HINT_DASHED?GREEN:hint==HINT_SOLID||hint==HINT_OCCUPIED?AMBER:MUTED;
    rounded(im,x,778,344,188,18,PANEL,225);
    rect(im,x+18,806,4,132,color,255);
    text(im,x+38,795,side?"右侧车道":"左侧车道",WHITE,1.f);
    text(im,x+38,840,hint==HINT_DASHED?"虚线 可考虑变道":hint==HINT_SOLID?"实线 保持车道":hint==HINT_OCCUPIED?"邻侧有目标 暂缓":"标线待确认",color,1.f);
    text(im,x+38,886,hint==HINT_DASHED?"请先确认侧后方":"保持观察 谨慎操作",MUTED,.78f);
    int direction=side?1:-1;
    if(turn==direction) {
        int on=((int)(phase*2)&1)==0;
        line(im,x+38,935,x+306,935,4,color,on?255:55);
    }
}
#endif
static const char *main_reason(const struct adas_status *s) {
#if HUD_UI_V2 && !defined(HUD_TEST_RAW_TEXT)
    if(intersection_live&&intersection_live->pedestrian_alarm)return "行人冲突 请停止";
#endif
    if(s->events&EVENT_RED_MOVING) return "红灯行驶 请停止";
    if(s->events&EVENT_SOLID_FORCE) return "强制跨实线变道";
    if(s->events&EVENT_PEDESTRIAN) return "前方行人";
    if(s->events&EVENT_APPROACH) return "前方目标快速逼近";
    if(s->events&EVENT_NEAR) return "前方目标过近";
    if(s->events&EVENT_CUT_IN) return "邻车切入候选 注意间距";
    if(s->events&EVENT_REAR_NEAR) return "后方来车 保持观察";
    if(s->events&EVENT_TURN_BLOCKED) {
#ifndef HUD_TEST_RAW_TEXT
        if(s->turn)return s->turn<0?"左转 变道受限":"右转 变道受限";
#endif
        return "当前变道条件受限";
    }
    if(s->events&EVENT_RED_STOP) return "红灯 请保持停止";
    if(s->events&EVENT_NO_LANE) return "车道待确认";
    if(s->events&EVENT_EDGE_LEFT) return "靠近左侧车道边界";
    if(s->events&EVENT_EDGE_RIGHT) return "靠近右侧车道边界";
    if(s->events&EVENT_DARK) return "暗区较多 注意观察";
    if(s->events&EVENT_OVERSPEED) return "模拟车速超过设定值";
    if(s->events&EVENT_STALE) return "帧数据已过期 暂停预警";
    if(s->hud_light==LIGHT_RED&&s->light!=LIGHT_RED) return "红灯候选 未确认ROI";
    if(s->events&EVENT_ZEBRA) return "斑马线候选";
    return "持续观察道路环境";
}
static const char *reason_en(const struct adas_status *s) {
    if(s->events&EVENT_RED_MOVING) return "RED LIGHT - STOP";
    if(s->events&EVENT_SOLID_FORCE) return "SOLID LINE CROSSING";
    if(s->events&EVENT_PEDESTRIAN) return "PEDESTRIAN AHEAD";
    if(s->events&EVENT_APPROACH) return "OBJECT APPROACHING";
    if(s->events&EVENT_NEAR) return "OBJECT TOO CLOSE";
    if(s->events&EVENT_CUT_IN) return "CUT IN CANDIDATE";
    if(s->events&EVENT_REAR_NEAR) return "后方大目标候选";
    if(s->events&EVENT_TURN_BLOCKED) return "LANE CHANGE RESTRICTED";
    if(s->events&EVENT_RED_STOP) return "RED LIGHT - KEEP STOPPED";
    if(s->events&EVENT_NO_LANE) return "LANE POSITION UNKNOWN";
    if(s->events&(EVENT_EDGE_LEFT|EVENT_EDGE_RIGHT)) return "LANE EDGE";
    if(s->events&EVENT_DARK) return "LOW VISIBILITY";
    if(s->events&EVENT_OVERSPEED) return "SIMULATED OVERSPEED";
    if(s->events&EVENT_STALE) return "FRAME DELAY";
    if(s->hud_light==LIGHT_RED&&s->light!=LIGHT_RED) return "SEARCH RED CANDIDATE";
    if(s->events&EVENT_ZEBRA) return "CROSSING CANDIDATE";
    return "KEEP WATCHING THE ROAD";
}

/* Short English tokens for judge-readable capsules (competition HUD). */
static unsigned lane_class_color(enum lane_class cls,int held) {
    if(held) return MUTED;
    return cls==LANE_CLASS_DASHED?CYAN:cls==LANE_CLASS_SOLID?AMBER:cls==LANE_CLASS_NONE?WHITE:MUTED;
}
static const char *light_token(enum adas_light L) {
    return L==LIGHT_RED?"RED":L==LIGHT_GREEN?"GRN":L==LIGHT_YELLOW?"YEL":"-";
}
static unsigned light_token_color(enum adas_light L) {
    return L==LIGHT_RED?RED:L==LIGHT_GREEN?GREEN:L==LIGHT_YELLOW?AMBER:MUTED;
}
static void signal_lamps(uint8_t *im,int cx,int cy,int step,int radius,unsigned off,enum adas_light lamp) {
    for(int i=0;i<3;i++) {
        enum adas_light type=i==0?LIGHT_RED:i==1?LIGHT_YELLOW:LIGHT_GREEN;
        for(int yy=-radius;yy<=radius;yy++)for(int xx=-radius;xx<=radius;xx++)if(xx*xx+yy*yy<=radius*radius)
            blend(im,cx+i*step+xx,cy+yy,light_token_color(type),lamp==type?255:off);
    }
}
const char *adas_hud_event_name(const struct adas_status *s) {
    /* Events can precede level confirmation; never hide them behind OK. */
    if(s->events&EVENT_RED_MOVING) return "RED+MOVE";
    if(s->events&EVENT_SOLID_FORCE) return "FORCE";
    if(s->events&EVENT_PEDESTRIAN) return "PED";
    if(s->events&EVENT_APPROACH) return "NEAR";
    if(s->events&EVENT_NEAR) return "NEAR";
    if(s->events&EVENT_CUT_IN) return "CUT IN";
    if(s->events&EVENT_REAR_NEAR) return "后方来车";
    if(s->events&EVENT_TURN_BLOCKED) return "BLOCK";
    if(s->events&EVENT_RED_STOP) return "RED STOP";
    if(s->events&EVENT_NO_LANE) return "NO LANE";
    if(s->events&(EVENT_EDGE_LEFT|EVENT_EDGE_RIGHT)) return "EDGE";
    if(s->events&EVENT_DARK) return "DARK";
    if(s->events&EVENT_OVERSPEED) return "SPEED";
    if(s->events&EVENT_STALE) return "STALE";
    if(s->hud_light==LIGHT_RED&&s->light!=LIGHT_RED) return "TL WATCH";
    if(s->events&EVENT_ZEBRA) return "CROSSING";
    if(s->level==ADAS_DANGER) return "DANGER";
    return s->level==ADAS_CAUTION?"CAUTION":"OK";
}
const char *adas_hud_lane_value_label(const struct adas_status *s,int side) {
#ifndef HUD_TEST_RAW_TEXT
    if(s->hud.enabled)return s->hud.type[side]==LANE_SOLID?"SOLID":s->hud.type[side]==LANE_DASHED?"DASHED":"UNKNOWN";
#endif
    enum lane_class cls=s->display_class[side];
    if(s->coasting_valid[side])
        cls=s->coasting_type[side]==LANE_SOLID?LANE_CLASS_SOLID:
            s->coasting_type[side]==LANE_DASHED?LANE_CLASS_DASHED:cls;
    if(s->lane_geometry) return adas_lane_class_name(cls);
    static const char *const labels[]={"NONE / NO GEO","DASHED / NO GEO","SOLID / NO GEO",
                                      "INVALID / NO GEO","UNKNOWN / NO GEO"};
    return cls>=LANE_CLASS_NONE&&cls<=LANE_CLASS_UNKNOWN?labels[cls]:"INVALID / NO GEO";
}
static void capsule(uint8_t *im,int x,int y,int w,int h,const char *label,const char *value,
                    unsigned value_color,unsigned fill) {
    rounded(im,x,y,w,h,10,fill,240);
    text(im,x+12,y+4,label,MUTED,.5f);
    float scale=.88f;
    char translated[1024];localize_hud(value,translated,sizeof(translated));
    int width=translated_width(translated,scale);
    if(width>w-24) scale*=((float)(w-24)/width);
    translated_text(im,x+12,y+24,translated,value_color,scale);
}
static void stroke_rect(uint8_t *im,int x,int y,int w,int h,unsigned c,unsigned a) {
    rect(im,x,y,w,1,c,a); rect(im,x,y+h-1,w,1,c,a);
    rect(im,x,y,1,h,c,a); rect(im,x+w-1,y,1,h,c,a);
}
static void hud_present(const struct adas_status *s,struct adas_status *display) {
    *display=*s;
#ifndef HUD_TEST_RAW_TEXT
    if(s->hud.enabled) {
        display->events=s->hud.events;display->level=s->hud.level;display->lane_geometry=s->hud.ready;
        for(int side=0;side<2;side++) {
            display->occupied[side]=s->hud.occupied[side];display->hints[side]=s->hud.hints[side];
            display->display_class[side]=s->hud.type[side]==LANE_SOLID?LANE_CLASS_SOLID:
                s->hud.type[side]==LANE_DASHED?LANE_CLASS_DASHED:LANE_CLASS_UNKNOWN;
            display->display_held[side]=0;
        }
    }
    /* Keep perception/state validity intact; omit only the pending-lane UI event. */
    display->events&=~EVENT_NO_LANE;
#endif
}
/* Shared road HUD has the route enum, not the parsed distance. Use the same
 * current-frame NAV policy; held cruise severity must not turn NEAR red. */
#if HUD_UI_V2 && !defined(HUD_TEST_RAW_TEXT)
static struct adas_nav_observation nav_status_route(const struct adas_status *s) {
    return (struct adas_nav_observation){.valid=s&&s->route>=-1&&s->route<=1,
        .direction=!s||s->route==2?NAV_UNKNOWN:s->route<0?NAV_LEFT:s->route>0?NAV_RIGHT:NAV_STRAIGHT};
}
#endif
static int nav_stopped(enum adas_nav_action action) {
    return action==NAV_STOP_RED||action==NAV_STOP_HAZARD;
}
static enum adas_hint hud_side_hint(const struct adas_status *s,int side,int quad) {
    return s->hud.enabled?s->hud.hints[side]:(s->lane_geometry||(quad&&s->occupied[side]))?s->hints[side]:HINT_UNKNOWN;
}
static const char *hud_side_text(enum adas_hint hint,int side) {
    return hint==HINT_DASHED?(side?"右道:虚线可借":"左道:虚线可借"):
        hint==HINT_SOLID?(side?"右道:实线保持":"左道:实线保持"):
        hint==HINT_OCCUPIED?(side?"右道:邻车暂缓":"左道:邻车暂缓"):(side?"右道:待确认":"左道:待确认");
}
static const char *hud_ego_text(const struct adas_status *s) {
    return s->ego_lane<0?"当前:左转道":s->ego_lane>0?"当前:右转道":"当前:直行道";
}
static void hud_center_text(const struct adas_status *s,char *output,size_t size) {
    int direction=s->turn?s->turn:s->simulated&&s->route>=-1&&s->route<=1?s->route:0;
    int blocked=s->hud.enabled?(s->events&EVENT_TURN_BLOCKED)!=0:
        s->turn&&(!s->lane_geometry||s->hints[s->turn>0]!=HINT_DASHED);
    int missing=s->hud.enabled?(s->events&EVENT_NO_LANE)!=0:!s->lane_geometry;
#ifndef HUD_TEST_RAW_TEXT
    missing=0;
#endif
    const char *action=s->events&EVENT_RED_MOVING?"红灯行驶 请停止":
        s->events&EVENT_SOLID_FORCE?"强制压实线 危险报警":s->events&EVENT_RED_STOP?"红灯亮起 保持停车":
        s->hud.turn_blocked?"变道受限 请勿强行":missing?"车道待确认 保持观察":blocked?"变道受限 请勿强行":
        s->turn?"转向提示 请确认侧后方":"保持车道 观察道路";
    if(s->hud.enabled&&!s->hud.turn_blocked&&(s->events&(EVENT_PEDESTRIAN|EVENT_APPROACH|EVENT_NEAR|EVENT_STALE|
       EVENT_DARK|EVENT_CUT_IN|EVENT_REAR_NEAR)))action=main_reason(s);
    snprintf(output,size,"%s | %s | %s",direction<0?"左转":direction>0?"右转":"直行",action,hud_ego_text(s));
}
int adas_hud_snapshot(const struct adas_status *raw,int quad,char *json,size_t capacity) {
    struct adas_status display;hud_present(raw,&display);const struct adas_status *s=&display;
    char center[256],left[128],right[128],event[128],adjacent[128],values[2][128];
    hud_center_text(s,center,sizeof(center));
    localize_hud(hud_side_text(hud_side_hint(s,0,quad),0),left,sizeof(left));
    localize_hud(hud_side_text(hud_side_hint(s,1,quad),1),right,sizeof(right));
    localize_hud(adas_hud_event_name(s),event,sizeof(event));
    for(int side=0;side<2;side++)localize_hud(adas_hud_lane_value_label(s,side),values[side],sizeof(values[side]));
    snprintf(adjacent,sizeof(adjacent),"左 %s | 右 %s",(s->lane_geometry||quad)?(s->occupied[0]?"有目标":"未见目标"):"未知",
        (s->lane_geometry||quad)?(s->occupied[1]?"有目标":"未见目标"):"未知");
    int banner=s->level!=ADAS_MONITOR||(s->events&EVENT_RED_STOP)||(s->hud_light==LIGHT_RED&&s->light!=LIGHT_RED);
    int n=snprintf(json,capacity,"{\"bottom_dock\":[\"%s\",\"%s\",\"%s\"],\"third_lane\":\"%s\",\"capsule_values\":[\"%s\",\"%s\"],\"event_text\":\"%s\",\"banner\":\"%s\",\"adjacent\":\"%s\",\"occupied\":[%d,%d],\"geo_ready\":%d,\"display_events\":%u,\"underlying_events\":%u,\"force_display_only\":%s,\"drawn_type\":[%d,%d]}",
        left,center,right,hud_ego_text(s),values[0],values[1],event,banner?main_reason(s):"",adjacent,
        s->occupied[0],s->occupied[1],s->lane_geometry,s->events,raw->events,raw->hud.force_from_hold?"1":"0",s->hud.type[0],s->hud.type[1]);
    return n<0||(size_t)n>=capacity?-1:0;
}
int adas_hud_intersection_snapshot(const struct adas_status *s,const struct intersection_view *v,int quad,char *json,size_t capacity) {
#if HUD_UI_V2 && !defined(HUD_TEST_RAW_TEXT)
    intersection_live=v;
    int rc=adas_hud_snapshot(s,quad,json,capacity);
    intersection_live=NULL;
    return rc;
#else
    (void)v;
    return adas_hud_snapshot(s,quad,json,capacity);
#endif
}
static void hud_objects(uint8_t *im,const struct features *f,const struct detection *d,int n,
    const struct adas_status *s,const struct track_view *tracks) {
    int quad=(f->source_kind&ADAS_SOURCE_QUAD)!=0;
    unsigned accent=s->level==ADAS_DANGER?RED:s->level==ADAS_CAUTION?AMBER:CYAN;
    int labels[MAX_DETECTIONS][4],label_count=0;
    for(int i=0;i<n;i++) {
        if(d[i].score<(s->hud.enabled?.35f:quad?.45f:.35f)||d[i].x1-d[i].x0<12||d[i].y1-d[i].y0<12) continue;
        if(!quad&&tracks&&tracks[i].id&&!tracks[i].confirmed&&d[i].score<.6f&&i!=s->lead_index) continue;
        int x0=(int)d[i].x0,y0=(int)d[i].y0,x1=(int)d[i].x1,y1=(int)d[i].y1;
        unsigned c=i==s->lead_index&&s->level!=ADAS_MONITOR?accent:(d[i].cls>=3?AMBER:CYAN);
        if(d[i].cls>=0&&d[i].cls<5) {
            int width=i==s->lead_index?3:2;
            line(im,x0,y0,x1,y0,width,c,255);line(im,x1,y0,x1,y1,width,c,255);
            line(im,x1,y1,x0,y1,width,c,255);line(im,x0,y1,x0,y0,width,c,255);
        } else corners(im,x0,y0,x1,y1,c,i==s->lead_index?3:2);
        if(quad||i==s->lead_index||d[i].score>=.6f) {
            char label[64]; if(intersection_app_render)snprintf(label,sizeof(label),"%s",class_cn(d[i].cls)); else snprintf(label,sizeof(label),"%s %.0f%%",class_cn(d[i].cls),d[i].score*100);
            int y=y0>=28?y0-26:y0+4;
            if(quad) {
                int width=text_width(label,.6f)+14;
                x0=x0<0?0:x0+width>=ADAS_W?ADAS_W-width-1:x0;
                int fy=adas_quad_front_y(f->source_kind),ry=adas_quad_rear_y(f->source_kind);
                int top=d[i].y0>=fy?fy:ry,bottom=d[i].y0>=fy?948:fy-28;
                y=y<top?top:y>bottom?bottom:y;
                for(int tries=0;tries<40;tries++) {
                    int collision=0;
                    for(int j=0;j<label_count;j++)if(x0<labels[j][2]&&x0+width>labels[j][0]&&
                        y<labels[j][3]&&y+24>labels[j][1]){collision=1;break;}
                    if(!collision)break;
                    y+=26;if(y>bottom){y=top;x0=(x0+width+8)%(ADAS_W-width);}
                }
                if(label_count<MAX_DETECTIONS){labels[label_count][0]=x0;labels[label_count][1]=y;
                    labels[label_count][2]=x0+width;labels[label_count++][3]=y+24;}
            }
            rounded(im,x0,y,text_width(label,.6f)+14,24,6,TOPBAR,quad?150:235);
            text(im,x0+7,y+2,label,c,.6f);
        }
    }

}

/* ==== HUD_UI_V2 CRUISE BEGIN ==== */
#if HUD_UI_V2 && !defined(HUD_TEST_RAW_TEXT)
#define CRUISE_PANEL 0x0e1624u
#define CRUISE_INFO 0x00e0c6u
#define CRUISE_WARN 0xffb840u
#define CRUISE_DANGER 0xff454fu
#define CRUISE_TEXT 0xebf2fau
#define CRUISE_MUTED 0x96aabeu
enum cruise_level { CRUISE_NONE=-1, CRUISE_INFO_LEVEL, CRUISE_WARN_LEVEL, CRUISE_DANGER_LEVEL };
struct cruise_alert {
    enum cruise_level level;
    uint32_t key;
    const char *title,*sub;
    int turn;
    double on,last,time;
    unsigned alpha;
};
static struct cruise_alert cruise_alert_state={.level=CRUISE_NONE};
static unsigned cruise_alpha(unsigned a,float coverage) {
    if(coverage<=0)return 0;
    return coverage>=1?a:(unsigned)(a*coverage+.5f);
}
/* One conversion per scanline, no allocation and no floating point per pixel. */
static void cruise_span(uint8_t *im,int x0,int x1,int y,unsigned color,unsigned alpha) {
    if(y<0||y>=ADAS_H||x1<x0||!alpha)return;
    if(x0<0)x0=0;
    if(x1>=ADAS_W)x1=ADAS_W-1;
    unsigned fg=color565(color),a=alpha+1,inv=256-a;
    unsigned fr=((fg>>11)&31)*a,fgreen=((fg>>5)&63)*a,fb=(fg&31)*a;
    for(int x=x0;x<=x1;x++) {
        uint8_t *p=im+((size_t)y*ADAS_W+x)*2;
        unsigned v=p[0]|(unsigned)p[1]<<8;
        unsigned value=alpha>=255?fg:(((((v>>11)&31)*inv+fr)>>8)<<11)|
            (((((v>>5)&63)*inv+fgreen)>>8)<<5)|(((v&31)*inv+fb)>>8);
        p[0]=(uint8_t)value;p[1]=(uint8_t)(value>>8);
    }
}
static void cruise_rounded(uint8_t *im,int x,int y,int w,int h,int r,unsigned color,unsigned alpha,unsigned border,unsigned ba) {
    for(int row=0;row<h;row++) {
        float dy=row<r?r-row-.5f:row>=h-r?row-(h-r)+.5f:0;
        float inset=dy>0?r-sqrtf(fmaxf(0,r*r-dy*dy)):0;
        float lo=x+inset,hi=x+w-1-inset;
        int a=(int)ceilf(lo),b=(int)floorf(hi);
        cruise_span(im,a,b,y+row,color,alpha);
        blend(im,a-1,y+row,color,cruise_alpha(alpha,a-lo));
        blend(im,b+1,y+row,color,cruise_alpha(alpha,hi-b));
        if(!row||row==h-1)cruise_span(im,a,b,y+row,border,ba);
        else {
            blend(im,a,y+row,border,ba);blend(im,b,y+row,border,ba);
            blend(im,a-1,y+row,border,cruise_alpha(ba,a-lo));
            blend(im,b+1,y+row,border,cruise_alpha(ba,hi-b));
        }
    }
}
/* Wu coverage for the two fractional edge pixels, extended to a finite-width stroke. */
static void cruise_line(uint8_t *im,float x0,float y0,float x1,float y1,float width,unsigned color,unsigned alpha) {
    if(!isfinite(x0)||!isfinite(y0)||!isfinite(x1)||!isfinite(y1)||width<=0)return;
    int steep=fabsf(y1-y0)>fabsf(x1-x0);
    float a0=steep?y0:x0,a1=steep?y1:x1,b0=steep?x0:y0,b1=steep?x1:y1;
    if(a0>a1){float t=a0;a0=a1;a1=t;t=b0;b0=b1;b1=t;}
    float gradient=(b1-b0)/fmaxf(1,a1-a0),half=.5f*width*sqrtf(1+gradient*gradient);
    int limit=steep?ADAS_H:ADAS_W;
    int first=(int)floorf(a0),last=(int)ceilf(a1);
    if(first<0)first=0;
    if(last>=limit)last=limit-1;
    for(int a=first;a<=last;a++) {
        float center=b0+(a-a0)*gradient,lo=center-half,hi=center+half;
        int low=(int)ceilf(lo),high=(int)floorf(hi),minor=steep?ADAS_W:ADAS_H;
        if(low<0)low=0;
        if(high>=minor)high=minor-1;
        for(int b=low;b<=high;b++)blend(im,steep?b:a,steep?a:b,color,alpha);
        if(low>0)blend(im,steep?low-1:a,steep?a:low-1,color,cruise_alpha(alpha,low-lo));
        if(high<minor-1)blend(im,steep?high+1:a,steep?a:high+1,color,cruise_alpha(alpha,hi-high));
    }
}
static const struct cruise_font *cruise_face(int size) {
    for(size_t i=0;i<sizeof(cruise_fonts)/sizeof(cruise_fonts[0]);i++)if(cruise_fonts[i].size==size)return &cruise_fonts[i];
    return &cruise_fonts[3];
}
static const struct cruise_glyph *cruise_glyph(const struct cruise_font *font,uint32_t code) {
    int lo=0,hi=font->count;
    while(lo<hi){int mid=(lo+hi)/2;if(font->glyphs[mid].code<code)lo=mid+1;else hi=mid;}
    return lo<font->count&&font->glyphs[lo].code==code?&font->glyphs[lo]:&font->glyphs['?'-32];
}
static int cruise_text_width(const char *value,int size) {
    const struct cruise_font *font=cruise_face(size);int width=0;
    const unsigned char *p=(const unsigned char *)value;
    while(*p)width+=cruise_glyph(font,next_utf8(&p))->advance;
    return width;
}
static void cruise_text(uint8_t *im,int x,int y,const char *value,int size,unsigned color,unsigned alpha,int max_width) {
    const struct cruise_font *font=cruise_face(size);int start=x;
    const unsigned char *p=(const unsigned char *)value;
    while(*p) {
        const struct cruise_glyph *g=cruise_glyph(font,next_utf8(&p));
        if(x+g->advance>start+max_width)break;
        const unsigned char *bitmap=font->alpha+g->offset;
        for(int row=0;row<font->h;row++)for(int col=0;col<font->w;col++)
            if(bitmap[row*font->w+col])blend(im,x+col,y+row,color,(bitmap[row*font->w+col]*alpha+127)/255);
        x+=g->advance;
    }
}
static void cruise_center(uint8_t *im,int cx,int y,const char *value,int size,unsigned color,unsigned alpha) {
    cruise_text(im,cx-cruise_text_width(value,size)/2,y,value,size,color,alpha,ADAS_W);
}
static inline void cruise_lane_aa_span(uint8_t *im,float lo,float hi,int y,unsigned color,unsigned alpha,
                                      const unsigned char blocked[ADAS_W]) {
    int first=(int)floorf(lo+.5f),last=(int)ceilf(hi-.5f);
    if(first<0)first=0;
    if(last>=ADAS_W)last=ADAS_W-1;
    for(int x=first;x<=last;x++)if(!blocked[x]) {
        float coverage=fminf(hi,x+.5f)-fmaxf(lo,x-.5f);
        if(coverage>0)blend(im,x,y,color,cruise_alpha(alpha,coverage));
    }
}
static inline void cruise_lane_chevron(uint8_t *im,float cx,float y,float width,float t,unsigned alpha,
                                      const struct detection *d,int n) {
    float half=width*.5f,rise=half*.42f,thickness=6+4*t;
    if(y+thickness>=972-16)return;
    const float px[6]={cx-half,cx,cx+half,cx+half,cx,cx-half};
    const float py[6]={y,y-rise,y,y+thickness,y-rise+thickness,y+thickness};
    for(int row=(int)floorf(y-rise);row<=(int)ceilf(y+thickness);row++) {
        float cuts[6];int count=0;
        for(int a=0,b=5;a<6;b=a++)if((py[a]>row)!=(py[b]>row))
            cuts[count++]=px[a]+(row-py[a])*(px[b]-px[a])/(py[b]-py[a]);
        for(int a=0;a<count;a++)for(int b=a+1;b<count;b++)if(cuts[b]<cuts[a]) {
            float swap=cuts[a];cuts[a]=cuts[b];cuts[b]=swap;
        }
        unsigned char blocked[ADAS_W]={0};
        for(int i=0;i<n;i++)if(d[i].score>=.35f&&row>=d[i].y0-5&&row<=d[i].y1+5) {
            int left=(int)fmaxf(0,d[i].x0-5),right=(int)fminf(ADAS_W-1,d[i].x1+5);
            if(left<=right)memset(blocked+left,1,(size_t)(right-left+1));
        }
        for(int a=0;a+1<count;a+=2)cruise_lane_aa_span(im,cuts[a],cuts[a+1],row,0xe6fffau,alpha,blocked);
    }
}
static inline __attribute__((unused)) void cruise_lane_raster(uint8_t *im,const float near_x[2],const float far_x[2],
                                     int far_y,int near_y,double phase_time,const struct detection *d,int n) {
    float span=(float)(near_y-far_y);
    float normal[2];
    for(int side=0;side<2;side++) {
        float slope=(near_x[side]-far_x[side])/span;
        normal[side]=sqrtf(1+slope*slope);
    }
    for(int y=far_y;y<=near_y;y++) {
        float t=(y-far_y)/span,x[2];
        for(int side=0;side<2;side++)x[side]=far_x[side]+(near_x[side]-far_x[side])*t;
        unsigned r=(unsigned)lroundf(58*(1-t)),g=(unsigned)lroundf(141+83*t),b=(unsigned)lroundf(255-57*t);
        unsigned color=(r<<16)|(g<<8)|b,alpha=(unsigned)lroundf(110*t);
        unsigned char blocked[ADAS_W]={0};
        for(int i=0;i<n;i++)if(d[i].score>=.35f&&y>=d[i].y0-5&&y<=d[i].y1+5) {
            int left=(int)fmaxf(0,d[i].x0-5),right=(int)fminf(ADAS_W-1,d[i].x1+5);
            if(left<=right)memset(blocked+left,1,(size_t)(right-left+1));
        }
        int first=(int)fmaxf(0,ceilf(x[0])),last=(int)fminf(ADAS_W-1,floorf(x[1]));
        while(first<=last) {
            while(first<=last&&blocked[first])first++;
            int begin=first;
            while(first<=last&&!blocked[first])first++;
            cruise_span(im,begin,first-1,y,color,alpha);
        }
        const float widths[3]={12,6,2};
        const unsigned alphas[3]={40,90,220},colors[3]={0x00e0c6u,0x00e0c6u,0xdcfffau};
        for(int layer=0;layer<3;layer++)for(int side=0;side<2;side++) {
            float half=.5f*widths[layer]*(.3f+.7f*t)*normal[side];
            cruise_lane_aa_span(im,x[side]-half,x[side]+half,y,colors[layer],alphas[layer],blocked);
        }
    }
    if(!isfinite(phase_time))phase_time=0;
    float depths[3];
    for(int k=0;k<3;k++) {
        depths[k]=.3f+.2f*k-(float)fmod(fmax(0,phase_time)/1.8,.6);
        while(depths[k]<.1f)depths[k]+=.6f;
    }
    for(int k=0;k<3;k++) {
        float t=depths[k];unsigned alpha=60;
        for(int j=0;j<3;j++)if(depths[j]<t)alpha+=50;
        float y=far_y+span*t;
        float left=far_x[0]+(near_x[0]-far_x[0])*t,right=far_x[1]+(near_x[1]-far_x[1])*t;
        cruise_lane_chevron(im,(left+right)*.5f,y,(right-left)*.35f,t,alpha,d,n);
    }
}
static void cruise_lane_smooth(struct lane fits[2],int usable[2],const int live[2],
                               const struct adas_status *s,float strength[2]) {
    static struct {
        unsigned generation,ready[2];
        enum lane_type type[2];
        double time,seen[2];
        float x[2][2],alpha[2];
    } view;
    double now=s->phase_time;
    if(!isfinite(now)){usable[0]=usable[1]=0;return;}
    if(view.generation!=g_hud_mode_generation||now<view.time||now-view.time>1) {
        memset(&view,0,sizeof(view));view.generation=g_hud_mode_generation;
    }
    float dt=(float)fmin(.2,fmax(0,now-view.time));view.time=now;
    float gain=1-expf(-dt/.45f),fade=1-expf(-dt/.25f),limit=120*dt;
    for(int side=0;side<2;side++) {
        float target[2]={fits[side].slope*704+fits[side].intercept,
                         fits[side].slope*963+fits[side].intercept};
        int found=usable[side]&&isfinite(target[0])&&isfinite(target[1])&&
            fabsf(target[0])<ADAS_W*2&&fabsf(target[1])<ADAS_W*2;
        if(found) {
            if(!view.ready[side]) {
                memcpy(view.x[side],target,sizeof(target));view.ready[side]=1;
            } else {
                float delta=fmaxf(fabsf(target[0]-view.x[side][0]),fabsf(target[1]-view.x[side][1]));
                float weight=delta<=1.5f?0:fminf(gain,limit/delta);
                for(int k=0;k<2;k++)view.x[side][k]+=(target[k]-view.x[side][k])*weight;
            }
            view.seen[side]=now;view.type[side]=fits[side].type;
        }
        float opacity=found?(live[side]?1:.41f):0;
        view.alpha[side]+=(opacity-view.alpha[side])*fade;strength[side]=view.alpha[side];
        usable[side]=view.ready[side]&&now-view.seen[side]<=.7&&strength[side]>=.03f;
        if(usable[side]) {
            fits[side].type=view.type[side];
            fits[side].slope=(view.x[side][1]-view.x[side][0])/(963-704);
            fits[side].intercept=view.x[side][0]-fits[side].slope*704;
        } else if(!found)view.ready[side]=0;
    }
}
static void cruise_road(uint8_t *im,const struct lane lanes[2],const struct adas_status *s,const struct detection *d,int n) {
    if((s->events&EVENT_STALE)||s->age_ms>250)return;
    static const unsigned char clear[ADAS_W]={0};
    const int first_y=704,last_y=972-9;
    struct lane fits[2];int live[2],usable[2];
    for(int side=0;side<2;side++) {
        fits[side]=lanes[side];live[side]=fits[side].valid&&fits[side].type!=LANE_UNKNOWN;
        if(!live[side]&&s->coasting_valid[side])fits[side]=(struct lane){.valid=1,.type=s->coasting_type[side],
            .slope=s->coasting_slope[side],.intercept=s->coasting_intercept[side]};
        if(s->hud.enabled&&fits[side].valid)fits[side].type=s->hud.type[side];
        usable[side]=fits[side].valid&&fits[side].type!=LANE_UNKNOWN&&isfinite(fits[side].slope)&&
            isfinite(fits[side].intercept)&&fabsf(fits[side].slope)<=2.35f;
    }
    float strength[2]={0};cruise_lane_smooth(fits,usable,live,s,strength);
    float fill_strength=fminf(strength[0],strength[1]);
    if(usable[0]&&usable[1])for(int y=first_y;y<=last_y;y++) {
        float left=fits[0].slope*y+fits[0].intercept,right=fits[1].slope*y+fits[1].intercept;
        if(left>=right||right<0||left>=ADAS_W)continue;
        float t=(float)(y-first_y)/(last_y-first_y);
        unsigned r=(unsigned)lroundf(58*(1-t)),g=(unsigned)lroundf(141+83*t),b=(unsigned)lroundf(255-57*t);
        unsigned color=(r<<16)|(g<<8)|b,alpha=(unsigned)lroundf(110*fill_strength*t);
        unsigned char blocked[ADAS_W]={0};
        for(int k=0;k<n;k++)if(d[k].score>=.35f&&y>=d[k].y0-5&&y<=d[k].y1+5) {
            int lo=(int)fmaxf(0,d[k].x0-5),hi=(int)fminf(ADAS_W-1,d[k].x1+5);
            if(lo<=hi)memset(blocked+lo,1,(size_t)(hi-lo+1));
        }
        int first=(int)fmaxf(0,ceilf(left)),last=(int)fminf(ADAS_W-1,floorf(right));
        while(first<=last) {
            while(first<=last&&blocked[first])first++;
            int begin=first;
            while(first<=last&&!blocked[first])first++;
            cruise_span(im,begin,first-1,y,color,alpha);
        }
    }
    for(int side=0;side<2;side++) {
        if(!usable[side])continue;
        struct lane fit=fits[side];
        unsigned color=fit.type==LANE_DASHED?CYAN:AMBER,opacity=(unsigned)lroundf(170*strength[side]);
        float half=.8f*sqrtf(1+fit.slope*fit.slope);
        for(int y=first_y;y<=last_y;y++) {
            if(fit.type==LANE_DASHED&&(y/32)%2)continue;
            float x=fit.slope*y+fit.intercept;
            if(x<0||x>=ADAS_W)continue;
            int masked=0;
            for(int k=0;k<n;k++)if(d[k].score>=.35f&&x>=d[k].x0-3&&x<=d[k].x1+3&&
                y>=d[k].y0-3&&y<=d[k].y1+3){masked=1;break;}
            if(!masked)cruise_lane_aa_span(im,x-half,x+half,y,color,opacity,clear);
        }
    }
    if(!intersection_app_render&&usable[0]&&usable[1])for(int k=0;k<3;k++) {
        float t=.3f+.2f*k,y=first_y+(last_y-first_y)*t;
        float left=fits[0].slope*y+fits[0].intercept,right=fits[1].slope*y+fits[1].intercept;
        if(left<0||right>=ADAS_W||left>=right)continue;
        unsigned alpha=(unsigned)(60+50*k);
        alpha=cruise_alpha(alpha,fill_strength);
        cruise_lane_chevron(im,(left+right)*.5f,y,(right-left)*.35f,t,alpha,d,n);
    }
}
static struct cruise_alert cruise_candidate(const struct adas_status *raw,int turns) {
    struct adas_status filtered;hud_present(raw,&filtered);
    filtered.events&=~hud_modes[g_hud_mode].suppressed;
    if(hud_modes[g_hud_mode].suppressed&EVENT_RED_MOVING)filtered.hud_light=filtered.light=LIGHT_UNKNOWN;
    if(intersection_live&&!intersection_live->pedestrian_alarm)filtered.events&=~EVENT_PEDESTRIAN;
    struct cruise_alert a={.level=CRUISE_NONE};
    static const uint32_t order[]={EVENT_SOLID_FORCE,EVENT_RED_MOVING,EVENT_PEDESTRIAN,EVENT_APPROACH,EVENT_NEAR,EVENT_RED_STOP,
        EVENT_CUT_IN,EVENT_REAR_NEAR,EVENT_TURN_BLOCKED,EVENT_EDGE_LEFT,EVENT_EDGE_RIGHT,
        EVENT_DARK,EVENT_OVERSPEED,EVENT_STALE,EVENT_ZEBRA};
    for(size_t i=0;i<sizeof(order)/sizeof(order[0]);i++)if(filtered.events&order[i]) {
        uint32_t bit=order[i];
        enum cruise_level level=bit&(EVENT_SOLID_FORCE|EVENT_RED_MOVING|EVENT_PEDESTRIAN|EVENT_APPROACH|EVENT_NEAR)?CRUISE_DANGER_LEVEL:
            bit&(EVENT_STALE|EVENT_ZEBRA)?CRUISE_INFO_LEVEL:CRUISE_WARN_LEVEL;
        if(level>a.level) {
            struct adas_status one=filtered;one.events=bit;
            a.level=level;a.key=bit;a.title=main_reason(&one);a.sub=reason_en(&one);
        }
    }
    if(turns&&filtered.turn) {
        int side=filtered.turn>0;
        int blocked=filtered.hud.enabled?filtered.hud.turn_blocked:
            !filtered.lane_geometry||filtered.hints[side]!=HINT_DASHED;
        enum cruise_level level=blocked?CRUISE_WARN_LEVEL:CRUISE_INFO_LEVEL;
        if(a.level<level||(a.key==EVENT_TURN_BLOCKED&&blocked)) {
            a=(struct cruise_alert){.level=level,.key=filtered.turn<0?0x10000000u:0x20000000u,
                .title=blocked?(side?"右转 变道受限":"左转 变道受限"):(side?"右转":"左转"),
                .sub=blocked?"变道受限 请勿强行":"转向提示 请确认侧后方",.turn=filtered.turn};
        }
    }
    return a;
}
static struct cruise_alert cruise_alert_update(const struct adas_status *s) {
    static unsigned generation;
    struct cruise_alert a=cruise_candidate(s,HUD_CRUISE_STAGE>=8),*held=&cruise_alert_state;
    if(generation!=g_hud_mode_generation){held->level=CRUISE_NONE;generation=g_hud_mode_generation;}
    double time=s->phase_time;
    if(!isfinite(time)||time<held->time||time-held->time>5)held->level=CRUISE_NONE;
    /* Turn cards and their edge light follow the current UI candidate together. */
    if(held->turn&&(a.turn!=held->turn||a.level!=held->level))held->level=CRUISE_NONE;
    if(intersection_live&&held->key==EVENT_PEDESTRIAN&&!intersection_live->pedestrian_alarm)held->level=CRUISE_NONE;
    if(held->level==CRUISE_NONE&&a.level!=CRUISE_NONE) {
        *held=a;held->on=held->last=time;
    } else if(a.level!=CRUISE_NONE&&a.key==held->key) {
        held->last=time;held->title=a.title;held->sub=a.sub;
    }
    else if(a.level>held->level||(a.turn&&held->turn&&a.turn!=held->turn)) {
        *held=a;held->on=held->last=time;
    }
    if(held->level!=CRUISE_NONE) {
        double until=held->level==CRUISE_DANGER_LEVEL?held->last+.6:
            fmax(held->on+(held->level==CRUISE_WARN_LEVEL?3:2),held->last);
        double opacity=fmin(1.,fmax(0.,(time-held->on)/.15));
        if(time>until)opacity*=fmax(0.,1-(time-until)/.25);
        held->alpha=(unsigned)(255*opacity+.5);
        if(time>=until+.25) {
            held->level=CRUISE_NONE;
            if(a.level!=CRUISE_NONE){*held=a;held->on=held->last=time;}
        }
    }
    held->time=time;return *held;
}
static void cruise_vignette(uint8_t *im,double phase,unsigned opacity,int top) {
    unsigned pulse=(unsigned)(opacity*(.75+.25*sinf((float)(phase*2.4))));
    for(int y=top;y<ADAS_H;y++) {
        int edge=y-top<(ADAS_H-top)/2?y-top:ADAS_H-1-y;
        float t=edge<86?1-edge/86.f:0;unsigned row_alpha=(unsigned)(140*t*t)*pulse/255;
        if(row_alpha)cruise_span(im,154,ADAS_W-155,y,CRUISE_DANGER,row_alpha);
        for(int x=0;x<154;x++) {
            float q=1-x/154.f;unsigned a=(unsigned)(140*q*q)*pulse/255;
            if(a<row_alpha)a=row_alpha;
            blend(im,x,y,CRUISE_DANGER,a);blend(im,ADAS_W-1-x,y,CRUISE_DANGER,a);
        }
    }
}
static void cruise_card(uint8_t *im,const struct cruise_alert *a,int offset) {
    if(a->level==CRUISE_NONE||!a->alpha)return;
    unsigned color=a->level==CRUISE_DANGER_LEVEL?CRUISE_DANGER:a->level==CRUISE_WARN_LEVEL?CRUISE_WARN:CRUISE_INFO;
    unsigned opacity=a->alpha;
    if(a->level==CRUISE_DANGER_LEVEL) {
        cruise_rounded(im,30,110+offset,532,116,28,color,30*opacity/255,color,20*opacity/255);
        cruise_rounded(im,33,113+offset,526,110,25,color,45*opacity/255,color,35*opacity/255);
    }
    cruise_rounded(im,36,116+offset,520,104,22,CRUISE_PANEL,190*opacity/255,0xffffffu,46*opacity/255);
    cruise_rounded(im,36,116+offset,520,104,22,CRUISE_PANEL,0,color,90*opacity/255);
    cruise_rounded(im,46,132+offset,8,72,4,color,opacity,color,0);
    if(a->turn||a->level==CRUISE_INFO_LEVEL) {
        cruise_rounded(im,72,146+offset,44,44,22,color,opacity,color,0);
        if(a->turn) {
            int sign=a->turn<0?-1:1;
            cruise_line(im,94-sign*12,168+offset,94+sign*12,168+offset,7,CRUISE_PANEL,opacity);
            cruise_line(im,94+sign*12,168+offset,94+sign*2,158+offset,7,CRUISE_PANEL,opacity);
            cruise_line(im,94+sign*12,168+offset,94+sign*2,178+offset,7,CRUISE_PANEL,opacity);
        } else {
            cruise_line(im,94,166+offset,94,180+offset,4,CRUISE_PANEL,opacity);
            cruise_rounded(im,92,156+offset,4,4,2,CRUISE_PANEL,opacity,CRUISE_PANEL,0);
        }
    } else {
        int sides=a->level==CRUISE_WARN_LEVEL?3:8;
        float px[8],py[8];
        for(int i=0;i<sides;i++) {
            float angle=-1.5707963f+(float)i*6.2831853f/sides;
            px[i]=94+22*cosf(angle);py[i]=168+offset+22*sinf(angle);
        }
        if(sides==3){px[0]=94;py[0]=146+offset;px[1]=116;py[1]=190+offset;px[2]=72;py[2]=190+offset;}
        for(int y=146+offset;y<=190+offset;y++) {
            float lo=1920,hi=-1;
            for(int i=0;i<sides;i++) {
                int j=(i+1)%sides;
                if((py[i]<=y&&py[j]>y)||(py[j]<=y&&py[i]>y)) {
                    float x=px[i]+(y-py[i])*(px[j]-px[i])/(py[j]-py[i]);
                    lo=fminf(lo,x);hi=fmaxf(hi,x);
                }
            }
            if(hi>=lo)cruise_span(im,(int)ceilf(lo),(int)floorf(hi),y,color,opacity);
        }
        for(int i=0;i<sides;i++)cruise_line(im,px[i],py[i],px[(i+1)%sides],py[(i+1)%sides],2,color,opacity);
        cruise_line(im,94,158+offset,94,170+offset,4,CRUISE_PANEL,opacity);
        cruise_rounded(im,92,176+offset,4,4,2,CRUISE_PANEL,opacity,CRUISE_PANEL,0);
    }
    const char *level=a->level==CRUISE_DANGER_LEVEL?"危险":a->level==CRUISE_WARN_LEVEL?"注意":"提示";
    int tw=cruise_text_width(level,14),label_x=556-tw-38;
    cruise_rounded(im,label_x,132+offset,tw+22,26,13,color,50*opacity/255,color,170*opacity/255);
    cruise_center(im,label_x+(tw+22)/2,138+offset,level,14,color,opacity);
    cruise_text(im,134,132+offset,a->title,32,color,opacity,label_x-146);
    char sub[512];localize_hud(a->sub,sub,sizeof(sub));
    cruise_text(im,134,180+offset,sub,18,0xc8d2deu,opacity,400);
}
static int cruise_straight_allowed(const struct adas_status *s) {
    uint32_t hazards=EVENT_STALE|EVENT_PEDESTRIAN|EVENT_NEAR|EVENT_APPROACH|EVENT_SOLID_FORCE|
        EVENT_RED_MOVING|EVENT_RED_STOP|EVENT_TURN_BLOCKED|EVENT_REAR_NEAR|EVENT_CUT_IN;
    hazards&=~hud_modes[g_hud_mode].suppressed;
    return !s->simulated&&!s->turn&&s->age_ms<=250&&s->level==ADAS_MONITOR&&!(s->events&hazards)&&
        s->light==LIGHT_GREEN&&s->signal.source==SIGNAL_ROI&&s->signal.stable&&
        s->signal.candidate==LIGHT_GREEN&&s->signal.box.valid&&s->signal.confidence>=.72f;
}

void adas_hud_pixel_light_mark(uint8_t *im,int x0,int y0,int x1,int y1,enum adas_light light,float confidence,int intersection) {
    if(!im||light==LIGHT_UNKNOWN||x0>x1||y0>y1)return;
    unsigned color=light==LIGHT_RED?0xff454fu:light==LIGHT_YELLOW?0xffb840u:0x3cdc78u;
    line(im,x0,y0,x1,y0,2,color,235);line(im,x1,y0,x1,y1,2,color,235);
    line(im,x1,y1,x0,y1,2,color,235);line(im,x0,y1,x0,y0,2,color,235);
    const char *name=light==LIGHT_RED?"红灯":light==LIGHT_YELLOW?"黄灯":"绿灯";
    char label[64];if(intersection||confidence<=0)snprintf(label,sizeof(label),"%s",name);else snprintf(label,sizeof(label),"%s %.0f%%",name,confidence*100);
    int lw=text_width(label,.6f)+14,ly=y0>=26?y0-26:y0+4,lx=x1+6;
    if(lx+lw>ADAS_W-2)lx=x0-lw-6;
    if(lx<2)lx=2;
    if(lx>ADAS_W-lw-2)lx=ADAS_W-lw-2;
    rounded(im,lx,ly,lw,24,6,TOPBAR,235);text(im,lx+7,ly+2,label,color,.6f);
}

static void cruise_signal_card(uint8_t *im,const struct adas_status *s,int offset,int warning_visible) {
    if(pixel_light_hud)return;
    if(s->signal.source==SIGNAL_SEARCH||s->age_ms>250||(s->events&EVENT_STALE)||!s->signal.stable||!s->signal.box.valid)return;
    const struct candidate *box=&s->signal.box;
    if(box->x0>=box->x1||box->y0>=box->y1||box->x1>=ADAS_W||box->y1>=ADAS_H)return;
    enum adas_light light=s->signal.candidate;
    if(light==LIGHT_UNKNOWN)return;
    unsigned color=light==LIGHT_RED?CRUISE_DANGER:light==LIGHT_YELLOW?CRUISE_WARN:CRUISE_INFO;
    if(g_hud_mode==HUD_MODE_NAV&&light==LIGHT_RED) {
        struct adas_nav_observation route=nav_status_route(s);
        if(adas_hud_nav_action(&route,s)!=NAV_STOP_RED)color=CRUISE_WARN;
    }
    cruise_line(im,box->x0,box->y0,box->x1,box->y0,2,color,230);
    cruise_line(im,box->x1,box->y0,box->x1,box->y1,2,color,230);
    cruise_line(im,box->x1,box->y1,box->x0,box->y1,2,color,230);
    cruise_line(im,box->x0,box->y1,box->x0,box->y0,2,color,230);
    const char *name=light==LIGHT_RED?"红灯":light==LIGHT_YELLOW?"黄灯":"绿灯";
    int label_x=box->x1+8<ADAS_W-120?(int)box->x1+8:(int)box->x0-80;
    cruise_text(im,label_x,(int)box->y0-4,name,22,color,255,120);
    if(warning_visible||!cruise_straight_allowed(s))return;
    int top=236+offset;
    cruise_rounded(im,36,top,520,104,22,CRUISE_PANEL,190,CRUISE_INFO,90);
    cruise_rounded(im,46,top+16,8,72,4,CRUISE_INFO,255,CRUISE_INFO,0);
    for(int y=0;y<=24;y++)cruise_span(im,94-y,94+y,top+24+y,CRUISE_INFO,255);
    for(int y=top+46;y<top+78;y++)cruise_span(im,87,101,y,CRUISE_INFO,255);
    cruise_text(im,134,top+16,"可直行",32,CRUISE_INFO,255,360);
    cruise_text(im,134,top+64,"绿灯提示 请确认前方",18,0xc8d2deu,255,400);
}
static void cruise_lane_label(const struct lane lanes[2],const struct adas_status *s,int side,
                              enum adas_hint hint,char *label,size_t size) {
    enum lane_type type=s->hud.enabled?s->hud.type[side]:lanes[side].valid?lanes[side].type:
        s->coasting_valid[side]?s->coasting_type[side]:LANE_UNKNOWN;
    const char *kind=type==LANE_SOLID?"实线":type==LANE_DASHED?"虚线":"未知";
    const char *action=hint==HINT_OCCUPIED?"·邻车暂缓":type==LANE_SOLID?"保持":
        type==LANE_DASHED?"可借":"";
    snprintf(label,size,"%s:%s%s",side?"右道":"左道",kind,action);
}
static void cruise_draw(uint8_t *im,const struct features *f,const struct detection *d,int n,
    const struct lane lanes[2],const struct adas_status *s,int debug,const char *timing,const struct track_view *tracks) {
    cruise_debug=debug;n=n<0?0:n>MAX_DETECTIONS?MAX_DETECTIONS:n;
    if(!d)n=0;
    struct adas_status presented;hud_present(s,&presented);
    struct adas_nav_observation nav_route=nav_status_route(s);
    enum adas_nav_action nav_action=adas_hud_nav_action(&nav_route,s);
    if(g_hud_mode==HUD_MODE_NAV) {
        presented.level=nav_stopped(nav_action)?ADAS_DANGER:
            nav_action==NAV_CAUTION||nav_action==NAV_SLOW_NEAR?ADAS_CAUTION:ADAS_MONITOR;
    }
    int offset=(f->source_kind&ADAS_SOURCE_QUAD_QUARTER)?270:0;
    if(HUD_CRUISE_STAGE>=4&&g_hud_mode!=HUD_MODE_NAV)cruise_road(im,lanes,s,d,n);
    if(HUD_CRUISE_STAGE>=5&&!(s->events&EVENT_STALE)&&s->age_ms<=250)hud_objects(im,f,d,n,&presented,tracks);
    struct cruise_alert alert={.level=CRUISE_NONE};
    if(HUD_CRUISE_STAGE>=7) {
        if(g_hud_mode==HUD_MODE_NAV) {
            /* NAV warnings follow the current real state, without the cruise
             * alert hold/vignette or a held HUD event overriding route red. */
            if(nav_action!=NAV_GUIDE&&nav_action!=NAV_WAIT) {
                alert=(struct cruise_alert){.level=nav_stopped(nav_action)?CRUISE_DANGER_LEVEL:CRUISE_WARN_LEVEL,
                    .title=nav_action==NAV_SLOW_NEAR?"前车较近":adas_hud_nav_reason(&nav_route,s),
                    .sub=adas_hud_nav_reason(&nav_route,s),.alpha=255};
            }
        } else if(intersection_live&&(intersection_live->red_alarm||intersection_live->pedestrian_alarm)) {
            alert=(struct cruise_alert){.level=CRUISE_DANGER_LEVEL,
                .title=intersection_live->pedestrian_alarm?main_reason(s):"红灯行驶 请停止",
                .sub="按键行驶指令 · 停车请双键长按",.alpha=255};
        } else if(intersection_live&&intersection_live->motion==1&&
                  !(s->events&(EVENT_STALE|EVENT_NEAR|EVENT_APPROACH|EVENT_SOLID_FORCE|EVENT_TURN_BLOCKED|EVENT_CUT_IN|EVENT_REAR_NEAR))) {
            alert=(struct cruise_alert){.level=CRUISE_INFO_LEVEL,.title="停车指令已接收",
                .sub="按键演示 请继续观察道路",.alpha=255};
        } else alert=cruise_alert_update(s);
        if(alert.level==CRUISE_DANGER_LEVEL&&g_hud_mode!=HUD_MODE_NAV)cruise_vignette(im,s->phase_time,alert.alpha,offset);
    }
    cruise_rounded(im,36,30+offset,264,66,33,CRUISE_PANEL,150,0xffffffu,46);
    const char *mode_title=hud_modes[g_hud_mode].title;
    cruise_text(im,64,47+offset,mode_title,22,CRUISE_TEXT,255,152);
    cruise_rounded(im,232,52+offset,22,22,11,CRUISE_INFO,255,CRUISE_INFO,0);
    cruise_line(im,66,86+offset,118,86+offset,1,CRUISE_INFO,100);
    cruise_line(im,124,86+offset,142,86+offset,1,CRUISE_INFO,60);
    char number[32]="";float fps;
    if(timing&&sscanf(timing,"FPS %f",&fps)==1&&isfinite(fps)&&fps>=0&&fps<=999)snprintf(number,sizeof(number),"%.0f",fps);
    cruise_text(im,262,52+offset,number,16,0xa0f0e6u,255,36);
    if(!intersection_app_render&&HUD_CRUISE_STAGE>=6&&(hud_modes[g_hud_mode].panels&HUD_PANEL_DARK)) {
        cruise_rounded(im,1560,30+offset,324,134,26,CRUISE_PANEL,170,0xffffffu,46);
        cruise_text(im,1584,44+offset,"暗处增强",22,CRUISE_TEXT,255,176);
        cruise_line(im,1584,38+offset,1638,38+offset,1,CRUISE_INFO,100);
        unsigned c=CRUISE_INFO;
        cruise_rounded(im,1774,46+offset,88,30,15,c,60,c,200);
        cruise_center(im,1818,53+offset,"已开启",16,0xa0f0e6u,255);
        cruise_text(im,1584,88+offset,f->profile==3?"Retinex":"低照",16,CRUISE_MUTED,255,280);
        char value[64];snprintf(value,sizeof(value),"亮度 %.0f",s->gray_mean);
        cruise_text(im,1584,118+offset,value,22,CRUISE_INFO,255,136);
        snprintf(value,sizeof(value),"暗区 %.0f%%",s->dark_ratio*100);
        cruise_text(im,1724,118+offset,value,22,CRUISE_INFO,255,152);
        cruise_rounded(im,360,972,1200,52,14,CRUISE_PANEL,130,0xffffffu,46);
        cruise_line(im,704,987,704,1009,1,CRUISE_INFO,70);
        cruise_line(im,1216,987,1216,1009,1,CRUISE_INFO,70);
        for(int side=0;side<2;side++) {
            enum adas_hint hint=hud_side_hint(&presented,side,(f->source_kind&ADAS_SOURCE_QUAD)!=0);
            unsigned color=hint==HINT_DASHED?0x3cdc78u:hint==HINT_SOLID||hint==HINT_OCCUPIED?CRUISE_WARN:CRUISE_MUTED;
            int bx=side?1264:374;
            cruise_rounded(im,bx,980,282,36,8,0x1e2634u,155,0xffffffu,0);
            cruise_span(im,bx+(side?276:2),bx+(side?279:5),984,color,240);
            for(int y=985;y<1012;y++)cruise_span(im,bx+(side?276:2),bx+(side?279:5),y,color,240);
            char label[96];cruise_lane_label(lanes,s,side,hint,label,sizeof(label));
            cruise_text(im,bx+16,987,label,22,color,255,250);
        }
    }
    if(HUD_CRUISE_STAGE>=7) {
        cruise_card(im,&alert,offset);
        if(alert.turn&&(alert.level==CRUISE_INFO_LEVEL||alert.level==CRUISE_WARN_LEVEL))
            hud_turn_edges(im,s,alert.level==CRUISE_INFO_LEVEL?GREEN:RED,alert.alpha);
        else if(alert.key==EVENT_TURN_BLOCKED)hud_turn_edges(im,s,RED,alert.alpha);
    }
    if(HUD_CRUISE_STAGE>=7&&(hud_modes[g_hud_mode].panels&HUD_PANEL_SIGNAL)&&!intersection_app_render) {
        unsigned c=s->hud_light==LIGHT_RED?CRUISE_DANGER:s->hud_light==LIGHT_YELLOW?CRUISE_WARN:
            s->hud_light==LIGHT_GREEN?CRUISE_INFO:CRUISE_MUTED;
        const char *light=s->hud_light==LIGHT_RED?"红灯":s->hud_light==LIGHT_YELLOW?"黄灯":
            s->hud_light==LIGHT_GREEN?"绿灯":"待确认";
        int unassigned_nav=g_hud_mode==HUD_MODE_NAV&&s->hud_light!=LIGHT_UNKNOWN&&
            (s->signal.source!=SIGNAL_ROI||s->light!=s->hud_light);
        if(g_hud_mode==HUD_MODE_NAV&&s->hud_light==LIGHT_RED&&nav_action!=NAV_STOP_RED)unassigned_nav=1;
        if(intersection_live&&s->hud_light!=LIGHT_UNKNOWN&&
           (s->signal.source!=SIGNAL_ROI||s->light==LIGHT_UNKNOWN||
            !isfinite(s->signal.confidence)||s->signal.confidence<.72f))unassigned_nav=1;
        if(unassigned_nav)c=CRUISE_WARN;
        char value[96];snprintf(value,sizeof(value),"信号灯 %s%s",light,s->signal.stable&&!unassigned_nav?"":"·候选");
        cruise_rounded(im,1560,178+offset,324,62,18,CRUISE_PANEL,170,c,80);
        cruise_text(im,1584,196+offset,value,22,c,255,280);
    }
    if(!intersection_app_render&&HUD_CRUISE_STAGE>=7&&(hud_modes[g_hud_mode].panels&HUD_PANEL_STRAIGHT)&&
       ((hud_modes[g_hud_mode].panels&HUD_PANEL_SIGNAL)||((f->source_kind&ADAS_SOURCE_QUAD_QUARTER)&&
       s->signal.source==SIGNAL_ROI&&s->signal.box.x0>=840&&s->signal.box.y0>=270&&
       s->signal.box.x1<1080&&s->signal.box.y1<480)))
        cruise_signal_card(im,s,offset,alert.level>=CRUISE_WARN_LEVEL);
    if(intersection_app_render&&!pixel_light_hud)cruise_signal_card(im,s,offset,1);
    if(s->simulated)cruise_text(im,320,48+offset,"模拟输入",16,CRUISE_WARN,255,144);
    if(debug) {
        char info[256];snprintf(info,sizeof(info),"帧 %u | 左 %s | 右 %s",s->source_id,
            adas_lane_source_name(s->lane_source[0]),adas_lane_source_name(s->lane_source[1]));
        text(im,40,1040,info,CRUISE_MUTED,.48f);
    }
}
#endif
/* ==== HUD_UI_V2 CRUISE END ==== */
void adas_hud_nav_bind_signal(struct adas_signal *signal,const struct adas_nav_observation *route,
    enum adas_nav_direction roi_direction) {
    if(signal&&signal->source==SIGNAL_ROI&&(!route||!route->valid||
       roi_direction==NAV_UNKNOWN||roi_direction!=adas_nav_current_direction(route)))signal->source=SIGNAL_SEARCH;
}
enum adas_nav_action adas_hud_nav_action(const struct adas_nav_observation *route,const struct adas_status *s) {
    if(!s||!isfinite(s->age_ms)||s->age_ms>250||(s->events&EVENT_STALE))return NAV_WAIT;
    /* A held red event alone cannot stop a newly parsed route. The current
     * detector must still confirm the explicitly route-bound ROI. */
    if(route&&route->valid&&adas_nav_current_control(route)==s->route&&
       s->light==LIGHT_RED&&s->signal.source==SIGNAL_ROI&&s->signal.stable&&
       s->signal.candidate==LIGHT_RED&&isfinite(s->signal.confidence)&&s->signal.confidence>=.72f&&
       (s->events&(EVENT_RED_STOP|EVENT_RED_MOVING)))return NAV_STOP_RED;
    if(s->level==ADAS_DANGER&&(s->events&(EVENT_PEDESTRIAN|EVENT_SOLID_FORCE|EVENT_TURN_BLOCKED)))return NAV_STOP_HAZARD;
    if(s->events&(EVENT_NEAR|EVENT_APPROACH))return NAV_SLOW_NEAR;
    if(s->level!=ADAS_MONITOR||s->hud_light==LIGHT_RED||s->hud_light==LIGHT_YELLOW||
       (s->signal.candidate!=LIGHT_UNKNOWN&&s->signal.source!=SIGNAL_ROI))return NAV_CAUTION;
    if(route&&route->valid&&!s->lane_geometry)return NAV_CAUTION;
    return route&&route->valid&&route->direction!=NAV_UNKNOWN?NAV_GUIDE:NAV_WAIT;
}
const char *adas_hud_nav_reason(const struct adas_nav_observation *route,const struct adas_status *s) {
    if(!s)return "保持观察";
    enum adas_nav_action action=adas_hud_nav_action(route,s);
    if((s->events&EVENT_STALE)||!isfinite(s->age_ms)||s->age_ms>250)return "帧数据已过期 暂停预警";
    if(action==NAV_STOP_RED)return "红灯亮起 保持停车";
    if(action==NAV_STOP_HAZARD) {
        struct adas_status hazard=*s;
        hazard.events&=EVENT_PEDESTRIAN|EVENT_SOLID_FORCE|EVENT_TURN_BLOCKED;
        return main_reason(&hazard);
    }
    if(action==NAV_SLOW_NEAR)return "前车较近 请减速 保持车距";
    if(s->hud_light==LIGHT_RED||s->signal.candidate==LIGHT_RED)return "红灯候选 保持观察";
    if(s->hud_light==LIGHT_YELLOW)return "黄灯亮起 留意信号";
    if(s->level!=ADAS_MONITOR)return main_reason(s);
    if(!route||!route->valid)return "保持观察";
    if(!s->lane_geometry)return "保持观察";
    return route->distance_m<=120?(route->direction==NAV_STRAIGHT?"保持车道 请确认路线":"准备转向 请确认车道"):
        route->distance_m<=500?"准备选道 请确认车道":"路线参考 保持当前车道";
}
static unsigned nav_action_color(enum adas_nav_action action) {
    return nav_stopped(action)?RED:action==NAV_CAUTION||action==NAV_SLOW_NEAR?AMBER:
        action==NAV_GUIDE?0x32e5c2u:MUTED;
}
/* NAV's broad translucent plane is bounded by current measured lines. The
 * label also avoids this mask, but moves/hides as a whole rather than clipping. */
static void nav_mask_row(unsigned char blocked[ADAS_W],int row,const struct detection *d,int n) {
    memset(blocked,0,ADAS_W);
    for(int i=0;i<n;i++)if(isfinite(d[i].score)&&d[i].score>=.25f&&
        isfinite(d[i].x0)&&isfinite(d[i].x1)&&isfinite(d[i].y0)&&isfinite(d[i].y1)&&
        d[i].x1>d[i].x0&&d[i].y1>d[i].y0&&row>=d[i].y0-5&&row<=d[i].y1+5) {
        int lo=(int)fmaxf(0,fminf(ADAS_W,ceilf(d[i].x0-5)));
        int hi=(int)fmaxf(-1,fminf(ADAS_W-1,floorf(d[i].x1+5)));
        if(hi>=lo)memset(blocked+lo,1,(size_t)(hi-lo+1));
    }
}
static void nav_span(uint8_t *im,float lo,float hi,int row,unsigned color,unsigned alpha,
    const unsigned char blocked[ADAS_W]) {
    int first=(int)fmaxf(0,ceilf(lo-.5f)),last=(int)fminf(ADAS_W-1,floorf(hi+.5f));
    for(int x=first;x<=last;x++)if(!blocked[x]) {
        float coverage=fminf(hi,x+.5f)-fmaxf(lo,x-.5f);
        if(coverage>0)blend(im,x,row,color,(unsigned)(alpha*fminf(1,coverage)));
    }
}
static void nav_road_label(uint8_t *im,const char *value,const struct lane lanes[2],
    const struct detection *objects,int count) {
#if HUD_UI_V2 && !defined(HUD_TEST_RAW_TEXT)
    const struct cruise_font *font=cruise_face(32);
    const int preferred_top=790;
    float left=lanes[0].slope*preferred_top+lanes[0].intercept;
    float right=lanes[1].slope*preferred_top+lanes[1].intercept;
    int text_width=cruise_text_width(value,32);
    float scale=fminf(2.f,(right-left)*.62f/fmaxf(1,(float)text_width));
    int height=(int)(font->h*scale),width=(int)(text_width*scale);
    /* Bitmap cells can overhang their advances, especially the last letter
     * in STOP. Reserve the complete raster footprint before drawing anything. */
    const unsigned char *text=(const unsigned char *)value;
    int advance=0,raster_width=0;
    while(*text) {
        const struct cruise_glyph *g=cruise_glyph(font,next_utf8(&text));
        int end=(int)((advance+font->w)*scale);
        if(end>raster_width)raster_width=end;
        advance+=g->advance;
    }
    if(height<=0||raster_width<=0)return;
    int top=-1,start=0;
    /* Keep the existing size/default anchor. If blocked, search down the
     * measured lane center for a complete slot below the foreground object. */
    for(int candidate=preferred_top;candidate+height<=961;candidate++) {
        float l0=lanes[0].slope*candidate+lanes[0].intercept;
        float r0=lanes[1].slope*candidate+lanes[1].intercept;
        float l1=lanes[0].slope*(candidate+height-1)+lanes[0].intercept;
        float r1=lanes[1].slope*(candidate+height-1)+lanes[1].intercept;
        float lo=fmaxf((l0+r0)*.5f-(r0-l0)*.4f,(l1+r1)*.5f-(r1-l1)*.4f);
        float hi=fminf((l0+r0)*.5f+(r0-l0)*.4f,(l1+r1)*.5f+(r1-l1)*.4f);
        int x=(int)((l0+r0-width)*.5f),last=x+raster_width-1;
        if(x<0||last>=ADAS_W||x<lo||last>hi)continue;
        int blocked=0;
        for(int i=0;i<count;i++)if(isfinite(objects[i].score)&&objects[i].score>=.25f&&
            isfinite(objects[i].x0)&&isfinite(objects[i].x1)&&
            isfinite(objects[i].y0)&&isfinite(objects[i].y1)&&
            objects[i].x1>objects[i].x0&&objects[i].y1>objects[i].y0&&
            last>=objects[i].x0-5&&x<=objects[i].x1+5&&
            candidate+height-1>=objects[i].y0-5&&candidate<=objects[i].y1+5) {
            blocked=1;break;
        }
        if(!blocked){top=candidate;start=x;break;}
    }
    if(top<0)return; /* No complete slot: never leave a partial word. */
    for(int row=0;row<height;row++) {
        int y=top+row,source_row=(int)(row/scale);
        const unsigned char *p=(const unsigned char *)value;advance=0;
        while(*p) {
            const struct cruise_glyph *g=cruise_glyph(font,next_utf8(&p));
            int a=(int)(advance*scale),b=(int)((advance+font->w)*scale);
            for(int x=a;x<b;x++) {
                int dest=start+x,col=(int)((x-advance*scale)/scale);
                if(col<0||col>=font->w)continue;
                unsigned alpha=font->alpha[g->offset+source_row*font->w+col];
                if(alpha)blend(im,dest,y,WHITE,alpha);
            }
            advance+=g->advance;
        }
    }
#else
    (void)im;(void)value;(void)lanes;(void)objects;(void)count;
#endif
}
void adas_hud_nav_road(uint8_t *im,const struct adas_nav_observation *route,
    const struct lane lanes[2],const struct adas_status *s,const struct detection *objects,int count) {
    if(!im||!s||!lanes||!s->lane_geometry)return;
    enum adas_nav_action action=adas_hud_nav_action(route,s);
    if(action==NAV_WAIT||((!route||!route->valid)&&!nav_stopped(action)))return;
    for(int side=0;side<2;side++)if(!lanes[side].valid||lanes[side].type==LANE_UNKNOWN||
        !isfinite(lanes[side].slope)||!isfinite(lanes[side].intercept))return;
    if(!objects||count<0)count=0;
    if(count>MAX_DETECTIONS)count=MAX_DETECTIONS;
    /* Project only inside both current measured lines; never extrapolate a
     * curved turn, use held/coasting geometry, or draw on the retained map. */
    for(int y=704;y<=960;y++) {
        float left=lanes[0].slope*y+lanes[0].intercept,right=lanes[1].slope*y+lanes[1].intercept;
        if(!isfinite(left)||!isfinite(right)||left<0||right>=ADAS_W||right-left<50)return;
    }
    int stopped=nav_stopped(action);
    unsigned color=nav_action_color(action),arrow_color=action==NAV_CAUTION||action==NAV_SLOW_NEAR?0xffce74u:0x50f6d6u;
    float px[2][6],py[2][6];
    if(!stopped)for(int k=0;k<2;k++) {
        double phase=isfinite(s->phase_time)?fmax(0,s->phase_time):0;
        /* Sparse chunky mint glyphs, no white core or artificial turn bend.
         * Near advisories remain calm/static while route red has no arrows. */
        float t=.13f+.57f*k+(action==NAV_GUIDE?.018f*sinf((float)(phase*2)):0);
        float y=704+256*t,left=lanes[0].slope*y+lanes[0].intercept;
        float right=lanes[1].slope*y+lanes[1].intercept,cx=(left+right)*.5f;
        float half=(right-left)*.33f,rise=20+12*t,thickness=20+10*t;
        const float xs[6]={cx-half,cx,cx+half,cx+half,cx,cx-half};
        const float ys[6]={y+rise,y,y+rise,y+rise+thickness,y+thickness,y+rise+thickness};
        memcpy(px[k],xs,sizeof(xs));memcpy(py[k],ys,sizeof(ys));
    }
    for(int y=704;y<=960;y++) {
        float left=lanes[0].slope*y+lanes[0].intercept,right=lanes[1].slope*y+lanes[1].intercept;
        float center=(left+right)*.5f,half=(right-left)*.4f,t=(float)(y-704)/256;
        float lo=center-half,hi=center+half;
        unsigned char blocked[ADAS_W];nav_mask_row(blocked,y,objects,count);
        nav_span(im,lo,hi,y,color,(unsigned)(60+35*t),blocked);
        if(stopped) {
            /* Static transverse bars replace ALL forward arrows and animation. */
            if((y>=886&&y<=903)||(y>=926&&y<=947))nav_span(im,lo,hi,y,RED,250,blocked);
            continue;
        }
        for(int k=0;k<2;k++) {
            float cuts[6];int nc=0;
            for(int a=0,b=5;a<6;b=a++)if((py[k][a]>y)!=(py[k][b]>y))
                cuts[nc++]=px[k][a]+(y-py[k][a])*(px[k][b]-px[k][a])/(py[k][b]-py[k][a]);
            for(int a=0;a<nc;a++)for(int b=a+1;b<nc;b++)if(cuts[b]<cuts[a]) {
                float swap=cuts[a];cuts[a]=cuts[b];cuts[b]=swap;
            }
            for(int a=0;a+1<nc;a+=2)nav_span(im,fmaxf(lo,cuts[a]),fminf(hi,cuts[a+1]),y,arrow_color,235,blocked);
        }
    }
    /* A distant maneuver stays in the top card; the road plane guides the
     * current lane until the near-turn window, without inventing a bend. */
    const char *label=stopped?"STOP":route->distance_m>120?"直行":
        route->direction==NAV_LEFT?"左转":route->direction==NAV_RIGHT?"右转":"直行";
    nav_road_label(im,label,lanes,objects,count);
}
/* Screen-space navigation reference, independent of measured lane validity.
 * Never feeds lane_geometry, lane classes, risk regions or distance estimates. */
static unsigned nav_ground_gray(const uint8_t *im,int x,int y) {
    unsigned p=im[2u*(y*ADAS_W+x)]|(unsigned)im[2u*(y*ADAS_W+x)+1]<<8;
    return (299*((p>>11)*255/31)+587*(((p>>5)&63)*255/63)+114*((p&31)*255/31))/1000;
}
static void nav_ground_window(const uint8_t *im,const struct detection *objects,int count,int *top,int *bottom) {
    int hood=(int)(.90f*ADAS_H);*top=(int)(.68f*ADAS_H);
    for(int y=(int)(.76f*ADAS_H);y<ADAS_H-10;y+=2) {
        unsigned above=0,below=0,dark=0,samples=0;
        for(int dy=0;dy<10;dy+=2)for(int x=(int)(.25f*ADAS_W);x<(int)(.60f*ADAS_W);x+=8) {
            unsigned b=nav_ground_gray(im,x,y+dy);
            above+=nav_ground_gray(im,x,y-15+dy);below+=b;dark+=b<65;samples++;
        }
        if(dark>.65f*samples&&(float)above-(float)below>24.f*samples){hood=y;break;}
    }
    *bottom=hood-(int)(.025f*ADAS_H);
    for(int i=0;i<count;i++)if(isfinite(objects[i].score)&&objects[i].score>=.25f&&
        isfinite(objects[i].x0)&&isfinite(objects[i].x1)&&isfinite(objects[i].y0)&&isfinite(objects[i].y1)&&
        objects[i].x1>objects[i].x0&&objects[i].y1>objects[i].y0&&
        objects[i].x1>.27f*ADAS_W&&objects[i].x0<.73f*ADAS_W&&objects[i].y1>.50f*ADAS_H) {
        int contact=(int)ceilf(objects[i].y1+.018f*ADAS_H);
        if(contact>*top)*top=contact;
    }
}
static void nav_ground_bounds(int y,int sign,int top,int bottom,float *low,float *high) {
    float t=(float)(bottom-y)/(bottom-top),center=ADAS_W*(.5f+sign*.075f*t*t);
    float half=ADAS_W*(.045f+.095f*(1-t))*.88f;
    *low=center-half;*high=center+half;
}
static int nav_ground_slot(int x,int y,int half,int depth,int pad,int sign,int top,int bottom,
    const struct detection *objects,int count,const int previous[4]) {
    int x0=x-half-pad,x1=x+half+pad,y0=y-depth-pad,y1=y+depth+pad;
    if(x0<0||x1>=ADAS_W||y0<top||y1>bottom)return 0;
    if(previous[0]>=0&&x0<previous[2]+8&&x1>previous[0]-8&&y0<previous[3]+5&&y1>previous[1]-5)return 0;
    for(int row=y0;row<=y1;row++) {
        float low,high;nav_ground_bounds(row,sign,top,bottom,&low,&high);
        if(x0<low||x1>high)return 0;
    }
    for(int i=0;i<count;i++)if(isfinite(objects[i].score)&&objects[i].score>=.25f&&
        isfinite(objects[i].x0)&&isfinite(objects[i].x1)&&isfinite(objects[i].y0)&&isfinite(objects[i].y1)&&
        objects[i].x1>objects[i].x0&&objects[i].y1>objects[i].y0&&
        x0<=objects[i].x1+6&&x1>=objects[i].x0-6&&y0<=objects[i].y1+6&&y1>=objects[i].y0-6)return 0;
    return 1;
}
void adas_hud_nav_reference(uint8_t *im,const struct adas_nav_observation *route,
    enum adas_nav_direction direction,const struct adas_status *s,const struct detection *objects,int count) {
    if(!im||!s||!route||!route->valid||direction==NAV_UNKNOWN)return;
    enum adas_nav_action action=adas_hud_nav_action(route,s);
    if(action==NAV_WAIT||nav_stopped(action))return;
    int bright=0;unsigned lo_gray=255,hi_gray=0;
    for(int y=600;y<950;y+=32)for(int x=200;x<1700;x+=32) {
        unsigned p=im[2u*(y*ADAS_W+x)]|(unsigned)im[2u*(y*ADAS_W+x)+1]<<8;
        unsigned gray=((p>>11)*8+((p>>5)&63)*4+(p&31)*8)/3;
        bright+=gray>20;if(gray<lo_gray)lo_gray=gray;if(gray>hi_gray)hi_gray=gray;
    }
    if(bright<10||hi_gray-lo_gray<16)return;
    if(!objects||count<0)count=0;
    if(count>MAX_DETECTIONS)count=MAX_DETECTIONS;
    int sign=direction==NAV_LEFT?-1:direction==NAV_RIGHT?1:0,top,bottom;
    nav_ground_window(im,objects,count,&top,&bottom);
    if(bottom-top<.028f*ADAS_H)return;
    unsigned color=action==NAV_SLOW_NEAR||action==NAV_CAUTION?AMBER:0x32e5c2u;
    for(int y=top;y<=bottom;y++) {
        float low,high;nav_ground_bounds(y,sign,top,bottom,&low,&high);
        unsigned char blocked[ADAS_W];nav_mask_row(blocked,y,objects,count);
        nav_span(im,low,high,y,color,78,blocked);
    }
    /* Whole flat marks: find a free rectangle inside this road ribbon. Do not
     * clip a chevron against a car and leave a broken or floating arrow. */
    int previous[4]={-1,0,0,0};const int sizes[4]={119,81,49,29};
    unsigned arrow=action==NAV_SLOW_NEAR||action==NAV_CAUTION?AMBER:0x62f8c1u;
    int marks=bottom-top<80?1:2;
    for(int k=0;k<marks;k++) {
        int placed=0,preferred=top+(int)((bottom-top)*(marks==1?.5f:k?.34f:.76f));
        for(int size=0;size<4&&!placed;size++) {
            int half=sizes[size],depth=(int)(half*.14f),thick=(int)(half*.09f);
            if(depth<5)depth=5;
            if(thick<3)thick=3;
            for(int offset=0;offset<=bottom-top&&!placed;offset+=3)for(int side=0;side<(offset?2:1)&&!placed;side++) {
                int y=preferred+(side?-offset:offset);
                if(y<top||y>bottom)continue;
                float low,high;nav_ground_bounds(y,sign,top,bottom,&low,&high);int center=(int)((low+high)/2);
                for(int dx=0;dx<(int)(high-low)&&!placed;dx+=4)for(int sx=0;sx<(dx?2:1)&&!placed;sx++) {
                    int x=center+(sx?-dx:dx),pad=thick+3;
                    if(!nav_ground_slot(x,y,half,depth,pad,sign,top,bottom,objects,count,previous))continue;
                    if(sign) {
                        line(im,x-sign*half/2,y-depth,x+sign*half/2,y,thick,arrow,245);
                        line(im,x+sign*half/2,y,x-sign*half/2,y+depth,thick,arrow,245);
                    } else {
                        line(im,x-half,y+depth,x,y-depth,thick,arrow,245);
                        line(im,x,y-depth,x+half,y+depth,thick,arrow,245);
                    }
                    previous[0]=x-half-pad;previous[1]=y-depth-pad;
                    previous[2]=x+half+pad;previous[3]=y+depth+pad;placed=1;
                }
            }
        }
    }
}
void adas_hud_nav_card(uint8_t *im,const struct adas_nav_observation *route,const struct adas_status *s) {
    if(!im||!route||!s)return;
    const int x=626,y=16,w=1262,h=102;
    enum adas_nav_action action=adas_hud_nav_action(route,s);
    unsigned accent=nav_action_color(action);
    int stopped=nav_stopped(action);
    if(stopped) {
        /* Screen-space alert on the composed road pane. No object masking,
         * route ribbon or tiny stop bars in the narrow gap beside a vehicle. */
        const int bx=937,by=854,bw=640,bh=124;
        rounded(im,bx,by,bw,bh,22,0x3d0c14u,242);
        stroke_rect(im,bx+2,by+2,bw-4,bh-4,RED,255);
        rounded(im,bx+20,by+20,82,82,16,RED,245);
        line(im,bx+35,by+61,bx+87,by+61,9,WHITE,255);
#if HUD_UI_V2 && !defined(HUD_TEST_RAW_TEXT)
        cruise_text(im,bx+126,by+18,action==NAV_STOP_RED?"红灯停车":"停车警示",32,RED,255,490);
        cruise_text(im,bx+126,by+72,"停止前行 保持观察",22,WHITE,255,490);
#else
        text(im,bx+126,by+18,"STOP",RED,1.f);
        text(im,bx+126,by+72,"停止前行 保持观察",WHITE,.7f);
#endif
    }
    rounded(im,x,y,w,h,22,PANEL,232);
    rect(im,x+20,y+20,4,h-40,accent,230);
    int ax=x+73,ay=y+52;
    if(stopped) {
        rounded(im,ax-30,ay-30,60,60,12,RED,230);
        line(im,ax-20,ay,ax+20,ay,8,WHITE,255);
    } else if(route->valid) {
        if(route->direction==NAV_STRAIGHT) {
            line(im,ax,ay+25,ax,ay-26,7,accent,255);
            line(im,ax,ay-26,ax-15,ay-9,7,accent,255);
            line(im,ax,ay-26,ax+15,ay-9,7,accent,255);
        } else {
            int direction=route->direction==NAV_LEFT?-1:1;
            line(im,ax-18*direction,ay+25,ax-18*direction,ay-15,7,accent,255);
            line(im,ax-18*direction,ay-15,ax+23*direction,ay-15,7,accent,255);
            line(im,ax+23*direction,ay-15,ax+8*direction,ay-30,7,accent,255);
            line(im,ax+23*direction,ay-15,ax+8*direction,ay,7,accent,255);
        }
    }
    char title[96];
    const char *direction=route->direction==NAV_LEFT?"左转":route->direction==NAV_RIGHT?"右转":"直行";
    if(stopped)snprintf(title,sizeof(title),"STOP");
    else if(route->valid)snprintf(title,sizeof(title),"%d m  %s",route->distance_m,direction);
    else snprintf(title,sizeof(title),"导航待确认");
    const char *stage=!route->valid?"导航提示":route->distance_m<=120?"近距提示":
        route->distance_m<=500?"准备选道":"路线参考";
    char stopped_stage[96];
    if(stopped&&route->valid) {
        snprintf(stopped_stage,sizeof(stopped_stage),"%d m  %s",route->distance_m,direction);
        stage=stopped_stage;
    }
    const char *hint=adas_hud_nav_reason(route,s);
#if HUD_UI_V2 && !defined(HUD_TEST_RAW_TEXT)
    cruise_text(im,x+128,y+11,title,32,WHITE,255,500);
    cruise_text(im,x+130,y+61,stage,18,MUTED,255,240);
    cruise_text(im,x+590,y+35,hint,22,accent,255,640);
#else
    (void)stage;
    text(im,x+128,y+17,title,WHITE,1.f);
    text(im,x+590,y+37,hint,accent,.8f);
#endif
}
void adas_hud_tracked(uint8_t *im,const struct features *f,const struct detection *d,int n,
              const struct lane lanes[2],const struct adas_status *s,int debug,const char *timing,
              const struct track_view *tracks) {
#if HUD_UI_V2 && !defined(HUD_TEST_RAW_TEXT)
    if(cruise_active()&&HUD_CRUISE_STAGE>=3) {
        cruise_draw(im,f,d,n,lanes,s,debug,timing,tracks);
        return;
    }
#endif
#ifndef HUD_TEST_RAW_TEXT
    enum adas_turn_feedback turn_feedback=adas_turn_feedback(s,lanes);
    struct adas_status display;
    struct lane display_lanes[2];
    if(s->hud.enabled) {
        hud_present(s,&display);
        for(int side=0;side<2;side++) {
            display_lanes[side]=lanes[side];display_lanes[side].valid=s->hud.side_valid[side];
            display_lanes[side].type=s->hud.type[side];
            if(s->coasting_valid[side]) {
                display_lanes[side].slope=s->coasting_slope[side];display_lanes[side].intercept=s->coasting_intercept[side];
            }
        }
        s=&display;lanes=display_lanes;
    }
#endif
    int quad=(f->source_kind&ADAS_SOURCE_QUAD)!=0;
    (void)debug; /* Legacy flag uses the same HUD; timing is already visible in the top bar. */
    unsigned accent=s->level==ADAS_DANGER?RED:s->level==ADAS_CAUTION?AMBER:CYAN;
    enum adas_light lamp=s->hud_light!=LIGHT_UNKNOWN?s->hud_light:
        (s->signal.stable?s->signal.candidate:LIGHT_UNKNOWN);
    const char *evt=adas_hud_event_name(s);

    if(s->lane_geometry) for(int y=740;y<(980);y+=2) {
        int left=(int)lroundf(lanes[0].slope*y+lanes[0].intercept);
        int right=(int)lroundf(lanes[1].slope*y+lanes[1].intercept);
        if(left>=0&&right<ADAS_W&&right>left) rect(im,left,y,right-left,1,CYAN,16);
    }
    for(int side=0;side<2;side++) {
        int live=lanes[side].valid&&lanes[side].type!=LANE_UNKNOWN;
        if(!live&&!s->coasting_valid[side]) continue;
        float slope=live?lanes[side].slope:s->coasting_slope[side];
        float intercept=live?lanes[side].intercept:s->coasting_intercept[side];
        enum lane_type type=live?lanes[side].type:s->coasting_type[side];
        if(type==LANE_UNKNOWN||!isfinite(slope)||!isfinite(intercept)) continue;
        unsigned c=type==LANE_DASHED?CYAN:AMBER;
        for(int y=704;y<(1020);y+=8) {
            if(type==LANE_DASHED&&(y/32)%2) continue;
            int x=(int)lroundf(slope*y+intercept);
            int next=(int)lroundf(slope*(y+8)+intercept);
            int masked=0;
            for(int i=0;i<n;i++)if(d[i].score>=.35f&&x>=d[i].x0-4&&x<=d[i].x1+4&&
                y+8>=d[i].y0-4&&y<=d[i].y1+4)masked=1;
            if(masked)continue; /* Do not extend a fitted road stripe through an observed vehicle. */
            if(x>=0&&x<ADAS_W&&next>=0&&next<ADAS_W) line(im,x,y,next,y+8,4,c,live?220:150);
        }
    }

    /* Preserve legacy paint underneath the translucent synchronization strip. */
    if(!quad&&s->lane_geometry) for(int side=0;side<2;side++) {
        if(!lanes[side].valid||lanes[side].type==LANE_UNKNOWN) continue;
        for(int y=1028;y<1040;y+=8) {
            if(lanes[side].type==LANE_DASHED&&(y/32)%2) continue;
            int x=(int)lroundf(lanes[side].slope*y+lanes[side].intercept);
            int next=(int)lroundf(lanes[side].slope*(y+8)+lanes[side].intercept);
            if(x>=0&&x<ADAS_W&&next>=0&&next<ADAS_W)
                line(im,x,y,next,y+8,2,lanes[side].type==LANE_DASHED?CYAN:AMBER,220);
        }
    }
    /* Fixed TinyCNN ROI outlines (roi_version=3): left 480,768 128x256 / right 1312,768. */
    if(!quad&&s->cnn_model_version==2&&s->cnn_roi_version==3&&!s->lane_geometry&&
       !s->coasting_valid[0]&&!s->coasting_valid[1]&&!s->roi_guided[0]&&!s->roi_guided[1]) {
        stroke_rect(im,480,768,128,256,CYAN,35);
        stroke_rect(im,1312,768,128,256,CYAN,35);
    }
    for(int side=0;side<2;side++)if(s->roi_guided[side]) {
        for(int y=s->roi_top[side]<704?704:s->roi_top[side];y<(1020);y+=8) {
            int a=(int)lroundf(s->roi_slope[side]*y+s->roi_intercept[side]);
            int b=(int)lroundf(s->roi_slope[side]*(y+8)+s->roi_intercept[side]);
            line(im,a-64,y,b-64,y+8,1,GREEN,110);
            line(im,a+63,y,b+63,y+8,1,GREEN,110);
        }
    }

    hud_objects(im,f,d,n,s,tracks);

    if(s->signal.source==SIGNAL_ROI&&s->signal.stable&&s->signal.box.valid&&lamp!=LIGHT_UNKNOWN) {
        const struct candidate *b=&s->signal.box;
        corners(im,(int)b->x0,(int)b->y0,(int)b->x1,(int)b->y1,light_token_color(lamp),2);
    }

    /* One dark strip: FPS | L | R | TL | EVT | OCC, then compact metrics. */
    rect(im,0,0,ADAS_W,80,TOPBAR,245);
    rect(im,0,80,ADAS_W,2,CYAN,200);
    text(im,24,14,"FPGA+NPU",MUTED,.55f);
    text(im,24,32,s->simulated?"ADAS SIM":"ADAS DEMO",WHITE,.85f);

    /* Parse FPS from timing "FPS xx.x | ..." when present. */
    char fps_buf[24]="-";
    if(timing&&strncmp(timing,"FPS ",4)==0) {
        const char *p=timing+4; size_t i=0;
        while(p[i]&&p[i]!=' '&&i+1<sizeof(fps_buf)) { fps_buf[i]=p[i]; i++; }
        fps_buf[i]=0;
    }
    capsule(im,192,10,120,60,"FPS",fps_buf,WHITE,CAPSULE);
    for(int side=0;side<2;side++) {
        char label[24]; snprintf(label,sizeof(label),"%c %s%s",side?'R':'L',s->hud.enabled?(s->display_cnn?"CPU":"RULE"):s->lane_source[side]==LANE_SOURCE_CPU_ROI?"CPU":s->display_cnn?"CNN":"RULE",s->hud.enabled?"":s->display_held[side]?" HOLD":"");
        capsule(im,328+side*196,10,180,60,label,adas_hud_lane_value_label(s,side),
                lane_class_color(s->display_class[side],s->display_held[side]),CAPSULE);
    }
    const char *light_source=s->signal.source==SIGNAL_MANUAL?"TL SIM":s->signal.source==SIGNAL_ROI?"TL ROI":"TL SEARCH";
    capsule(im,720,10,152,60,light_source,light_token(lamp),light_token_color(lamp),CAPSULE);
    char event_label[32]; snprintf(event_label,sizeof(event_label),"EVT / %s",adas_level_name(s->level));
    capsule(im,888,10,216,60,event_label,evt,!strcmp(evt,"OK")?MUTED:accent,CAPSULE);

    char adjacent[96];
    snprintf(adjacent,sizeof(adjacent),"L %s | R %s",(s->lane_geometry||quad)?(s->occupied[0]?"OCCUPIED":"NO OBJ"):"UNKNOWN",
             (s->lane_geometry||quad)?(s->occupied[1]?"OCCUPIED":"NO OBJ"):"UNKNOWN");
    rounded(im,1120,10,380,60,10,CAPSULE,240);
    text(im,1132,14,quad?"同帧四模块 / 侧后占用":"FRONT VIEW / ADJ",MUTED,.5f);
    text(im,1132,36,adjacent,s->occupied[0]||s->occupied[1]?AMBER:MUTED,.68f);
    const char *metrics=timing?timing:"";
    if(!strncmp(metrics,"FPS ",4)) {
        metrics=strchr(metrics,'|');
        if(metrics) { metrics++; while(*metrics==' ') metrics++; }
        else metrics="";
    }
    text(im,1516,28,metrics,MUTED,.58f);

#ifdef HUD_TEST_RAW_TEXT
    if(s->level!=ADAS_MONITOR||s->events&(EVENT_RED_STOP|EVENT_NO_LANE)||
       (s->hud_light==LIGHT_RED&&s->light!=LIGHT_RED)) {
        unsigned warning=s->level==ADAS_MONITOR?(s->events&EVENT_RED_STOP?RED:
            s->hud_light==LIGHT_RED&&s->light!=LIGHT_RED?AMBER:MUTED):accent;
        rounded(im,24,94,1120,88,10,TOPBAR,235);
        rect(im,24,94,5,88,warning,255);
        text(im,44,101,main_reason(s),warning,1.08f);
        text(im,44,149,reason_en(s),MUTED,.62f);
    }
    if(s->road_ready) {
        char advice[240];
        const char *profile=s->road_profile==ROAD_HIGHWAY?"HWY CONFIG":s->road_profile==ROAD_URBAN?"URBAN CONFIG":"AUTO CONTEXT";
        snprintf(advice,sizeof(advice),"%s | %s | %s",profile,adas_road_scene_name(s->road_scene),adas_road_action_name(s->road_action));
        rounded(im,24,192,1462,48,10,TOPBAR,225);
        text(im,40,199,advice,accent,.66f);
    }
    /* New widgets stay outside all protected pixel bands. */
    rounded(im,1510,94,386,150,14,TOPBAR,230);
    signal_lamps(im,1540,123,42,14,38,lamp);
    text(im,1668,103,lamp==LIGHT_RED?(s->light==LIGHT_RED?"红灯 停车提示":"红灯 候选待确认"):lamp==LIGHT_GREEN?"绿灯 观察通行":lamp==LIGHT_YELLOW?"黄灯 注意减速":"灯态 监测",light_token_color(lamp),.7f);
    char hardware[128];
    snprintf(hardware,sizeof(hardware),"%s 配置%u 模式%u | %s",f->profile==3?"Retinex":"LUX",f->profile,f->mode,s->auto_signal_fsm?"SEARCH LINK":"ROI FSM");
    text(im,1526,150,hardware,MUTED,.5f);
    snprintf(hardware,sizeof(hardware),"边缘像素 %u | 形态像素 %u",s->fpga_edge_pixels,s->fpga_morph_pixels);
    text(im,1526,177,hardware,CYAN,.5f);
    snprintf(hardware,sizeof(hardware),"Y %.0f | DARK %.0f%% | ZEBRA %s",s->gray_mean,s->dark_ratio*100,s->fpga_zebra_candidate?"CAND":"-");
    text(im,1526,204,hardware,MUTED,.5f);
    side_card(im,0,s->lane_geometry?s->hints[0]:HINT_UNKNOWN,s->turn,s->phase_time);
    side_card(im,1,s->lane_geometry?s->hints[1]:HINT_UNKNOWN,s->turn,s->phase_time);
    rounded(im,652,842,616,124,18,PANEL,230);
    int direction=s->turn?s->turn:s->simulated&&s->route>=-1&&s->route<=1?s->route:0;
    int blocked=s->turn&&(!s->lane_geometry||s->hints[s->turn>0]!=HINT_DASHED);
    unsigned arrow_color=s->level==ADAS_DANGER?RED:blocked?AMBER:s->turn?GREEN:CYAN;
    arrow(im,714,878,direction,arrow_color,!s->turn||((int)(s->phase_time*2)&1)==0);
    const char *action=s->events&EVENT_RED_MOVING?"红灯行驶 请停止":
        s->events&EVENT_SOLID_FORCE?"强制压实线 危险报警":
        s->events&EVENT_RED_STOP?"红灯亮起 保持停车":
        !s->lane_geometry?"车道待确认 保持观察":
        blocked?"变道受限 请勿强行":s->turn?"转向提示 请确认侧后方":"保持车道 观察道路";
    text(im,786,854,action,s->level==ADAS_DANGER?RED:WHITE,.85f);
    text(im,786,895,((!lanes[0].valid&&s->coasting_valid[0])||(!lanes[1].valid&&s->coasting_valid[1]))?"惯性显线·无几何禁强变":s->gap_veto[0]||s->gap_veto[1]?"长程间隙修正":s->ego_lane<0?"当前:左转道":s->ego_lane>0?"当前:右转道":"当前:直行道",MUTED,.65f);
    text(im,786,928,s->simulated?"模拟按键 J/K/L车道 M灯联动":"前视辅助 不覆盖侧后方盲区",MUTED,.52f);
#else
    if(s->level!=ADAS_MONITOR||(s->events&EVENT_RED_STOP)||
       (s->hud_light==LIGHT_RED&&s->light!=LIGHT_RED)) {
        unsigned warning=s->level==ADAS_MONITOR?(s->events&EVENT_RED_STOP?RED:AMBER):accent;
        rounded(im,24,94,468,62,10,TOPBAR,150);
        rect(im,24,94,4,62,warning,240);
        text(im,40,98,main_reason(s),warning,.84f);
        text(im,40,130,reason_en(s),MUTED,.50f);
    }
    rounded(im,1616,92,280,54,12,TOPBAR,135);
    signal_lamps(im,1638,119,26,9,42,lamp);
    text(im,1716,96,lamp==LIGHT_RED?(s->light==LIGHT_RED?"红灯 停车提示":"红灯 候选待确认"):
        lamp==LIGHT_GREEN?"绿灯 观察通行":lamp==LIGHT_YELLOW?"黄灯 注意减速":"灯态 监测",
        light_token_color(lamp),.58f);
    char hardware[96];
    snprintf(hardware,sizeof(hardware),"%s | Y %.0f | 暗区 %.0f%%",f->profile==3?"Retinex":"LUX",s->gray_mean,s->dark_ratio*100);
    float hardware_scale=.44f;
    int hardware_width=text_width(hardware,hardware_scale);
    if(hardware_width>168)hardware_scale*=168.f/hardware_width;
    text(im,1716,122,hardware,MUTED,hardware_scale);

    if(turn_feedback!=TURN_IDLE) {
        unsigned turn_color=turn_feedback==TURN_RED?RED:turn_feedback==TURN_GREEN?GREEN:AMBER;
        unsigned alpha=(((int)(s->phase_time*4)&1)==0)?240:55;
        rect(im,s->turn<0?8:1904,344,8,680,turn_color,alpha);
    }
    int dock_y=972;
    rounded(im,360,dock_y,1200,52,14,PANEL,130);
    for(int side=0;side<2;side++) {
        enum adas_hint hint=hud_side_hint(s,side,quad);
        unsigned c=hint==HINT_DASHED?GREEN:hint==HINT_SOLID||hint==HINT_OCCUPIED?AMBER:MUTED;
        int bx=side?1264:374;
        rounded(im,bx,dock_y+8,282,36,8,CAPSULE,155);
        rect(im,bx+(side?276:2),dock_y+12,4,28,c,240);
        const char *hstr=hud_side_text(hint,side);
        text(im,bx+16,dock_y+12,hstr,c,.62f);
        if(s->turn==(side?1:-1)&&(((int)(s->phase_time*2)&1)==0))
            line(im,bx+16,dock_y+40,bx+260,dock_y+40,3,c,255);
    }
    int blocked=s->turn&&(!s->lane_geometry||s->hints[s->turn>0]!=HINT_DASHED);
    unsigned arrow_color=s->level==ADAS_DANGER?RED:blocked?AMBER:s->turn?GREEN:CYAN;
    char center_line[256];hud_center_text(s,center_line,sizeof(center_line));
    float center_scale=.64f;
    int center_width=text_width(center_line,center_scale);
    if(center_width>584)center_scale*=584.f/center_width;
    centered(im,960,dock_y+12,center_line,s->level==ADAS_DANGER?RED:arrow_color,center_scale);
#endif
    if(quad) {
        int fy=adas_quad_front_y(f->source_kind),ry=adas_quad_rear_y(f->source_kind);
        rect(im,0,fy,1920,1,CYAN,150);
        for(int j=0;j<2;j++)for(int y=ry;y<fy;y+=28)
            rect(im,j?QUAD_RIGHT_SEAM:QUAD_LEFT_SEAM,y,1,7,MUTED,150);
        text(im,12,ry+4,"左后视角",s->occupied[0]?AMBER:CYAN,.58f);
        text(im,QUAD_LEFT_SEAM+16,ry+4,"正后视角",s->events&EVENT_REAR_NEAR?AMBER:CYAN,.58f);
        text(im,QUAD_RIGHT_SEAM+16,ry+4,"右后视角",s->occupied[1]?AMBER:CYAN,.58f);
    }
    if(s->level==ADAS_DANGER) {
        float pulse=.5f+.5f*sinf((float)(s->phase_time*9.42477796));
        unsigned alpha=(unsigned)(100+120*pulse);
        for(int edge=0;edge<8;edge++) {
            rect(im,edge,edge,ADAS_W-edge*2,1,RED,alpha);
            rect(im,edge,ADAS_H-1-edge,ADAS_W-edge*2,1,RED,alpha);
            rect(im,edge,edge,1,ADAS_H-edge*2,RED,alpha);
            rect(im,ADAS_W-1-edge,edge,1,ADAS_H-edge*2,RED,alpha);
        }
    }
#ifndef HUD_TEST_RAW_TEXT
    if(s->turn&&(s->events&EVENT_TURN_BLOCKED))hud_turn_edges(im,s,RED,255);
    else hud_turn_edges(im,s,GREEN,turn_feedback==TURN_GREEN&&s->level!=ADAS_DANGER?255:0);
#endif
}

void adas_hud_cnn(uint8_t *im,const struct adas_status *s,const char *sync,int fresh) {
#if HUD_UI_V2 && !defined(HUD_TEST_RAW_TEXT)
    if(cruise_active()&&!cruise_debug)return;
#endif
    struct adas_status display;hud_present(s,&display);s=&display;
    /* Both class readouts share the presented status, including held labels. */
    char status[256];
    snprintf(status,sizeof(status),"%s L %s%s  R %s%s | %s",s->display_cnn?"CNN":"RULE",
        adas_hud_lane_value_label(s,0),s->display_held[0]?" HOLD":"",
        adas_hud_lane_value_label(s,1),s->display_held[1]?" HOLD":"",sync?sync:"");
    rounded(im,24,ADAS_H-44,1872,32,8,TOPBAR,235);
    rect(im,24,ADAS_H-44,4,32,fresh?CYAN:MUTED,255);
    text(im,40,ADAS_H-40,status,fresh?CYAN:MUTED,.58f);
}
void adas_hud(uint8_t *im,const struct features *f,const struct detection *d,int n,
              const struct lane lanes[2],const struct adas_status *s,int debug,const char *timing) {
    adas_hud_tracked(im,f,d,n,lanes,s,debug,timing,NULL);
}

#if HUD_UI_V2 && !defined(HUD_TEST_RAW_TEXT)
struct intersection_arrow_display {
    enum intersection_permission permission[3],pending[3];
    double allowed_since[3],blocked_seen[3],last,command_seen;
    unsigned generation;
    int initialized,command,pedestrian_block;
};
static struct intersection_arrow_display intersection_arrows;
/* Display holds can retain a restriction, never a permission without a current
 * confirmed observation. They do not feed back into controls or events. */
static void intersection_arrow_update(struct intersection_arrow_display *ui,
    const struct intersection_view *v,const struct adas_status *s) {
    double t=s->phase_time;
    int reset=!ui->initialized||ui->generation!=g_hud_mode_generation||
        !isfinite(t)||t<ui->last||t-ui->last>.5;
    if(reset) {
        memset(ui,0,sizeof(*ui));ui->initialized=1;ui->generation=g_hud_mode_generation;
        for(int i=0;i<3;i++)ui->permission[i]=ui->pending[i]=JUNCTION_UNCONFIRMED;
    }
    ui->last=t;
    if(!isfinite(t)||v->stale||!isfinite(s->age_ms)||s->age_ms>250||(s->events&EVENT_STALE)) {
        for(int i=0;i<3;i++)ui->permission[i]=ui->pending[i]=JUNCTION_UNCONFIRMED;
        ui->command=0;return;
    }
    if(v->command==JUNCTION_STOP)ui->command=0;
    else if(v->command>=JUNCTION_LEFT&&v->command<=JUNCTION_RIGHT) {
        ui->command=v->command;ui->command_seen=t;
    } else if(t-ui->command_seen>.35)ui->command=0;
    for(int i=0;i<3;i++) {
        enum intersection_permission raw=v->permission[i];
        if(ui->pedestrian_block&&!v->pedestrian_present&&raw!=JUNCTION_BLOCKED)
            ui->permission[i]=ui->pending[i]=JUNCTION_UNCONFIRMED;
        if(raw==JUNCTION_BLOCKED) {
            ui->permission[i]=ui->pending[i]=JUNCTION_BLOCKED;ui->blocked_seen[i]=t;
        } else if(raw==JUNCTION_ALLOWED) {
            if(ui->pending[i]!=JUNCTION_ALLOWED)ui->allowed_since[i]=t;
            ui->pending[i]=JUNCTION_ALLOWED;
            if(ui->permission[i]==JUNCTION_BLOCKED&&t-ui->blocked_seen[i]<.5)continue;
            ui->permission[i]=t-ui->allowed_since[i]>=.3?JUNCTION_ALLOWED:JUNCTION_UNCONFIRMED;
        } else {
            ui->pending[i]=JUNCTION_UNCONFIRMED;
            if(ui->permission[i]!=JUNCTION_BLOCKED||t-ui->blocked_seen[i]>=.5)
                ui->permission[i]=JUNCTION_UNCONFIRMED;
        }
    }
    ui->pedestrian_block=v->pedestrian_present;
}
static void intersection_arrow_shape(uint8_t *im,int cx,int direction,unsigned color,int selected) {
    float px[48],py[48];int count=0;
#define ARROW_POINT(x,y) do { px[count]=cx+(direction?(direction*(x)):(x));py[count]=938+(y);count++; } while(0)
    ARROW_POINT(-9,0);
    if(!direction) {
        ARROW_POINT(-9,-112);ARROW_POINT(-32,-112);ARROW_POINT(0,-146);
        ARROW_POINT(32,-112);ARROW_POINT(9,-112);ARROW_POINT(9,0);
    } else {
        ARROW_POINT(-9,-58);
        for(int k=1;k<=10;k++) {
            float angle=3.14159265f+k*.157079633f;
            ARROW_POINT(48+57*cosf(angle),-58+57*sinf(angle));
        }
        ARROW_POINT(74,-115);ARROW_POINT(74,-133);ARROW_POINT(110,-106);
        ARROW_POINT(74,-79);ARROW_POINT(74,-97);ARROW_POINT(48,-97);
        for(int k=1;k<=10;k++) {
            float angle=4.71238898f-k*.157079633f;
            ARROW_POINT(48+39*cosf(angle),-58+39*sinf(angle));
        }
        ARROW_POINT(9,0);
    }
    for(int k=1;k<=8;k++) {
        float angle=k*.392699082f;
        ARROW_POINT(9*cosf(angle),9*sinf(angle));
    }
#undef ARROW_POINT
    static const unsigned char clear[ADAS_W]={0};
    for(int y=790;y<=949;y++) {
        float cuts[48];int ncuts=0;
        for(int a=0,b=count-1;a<count;b=a++)if((py[a]>y+.5f)!=(py[b]>y+.5f))
            cuts[ncuts++]=px[a]+(y+.5f-py[a])*(px[b]-px[a])/(py[b]-py[a]);
        for(int a=1;a<ncuts;a++)for(int b=a;b>0&&cuts[b]<cuts[b-1];b--) {
            float value=cuts[b];cuts[b]=cuts[b-1];cuts[b-1]=value;
        }
        for(int a=0;a+1<ncuts;a+=2)
            cruise_lane_aa_span(im,cuts[a],cuts[a+1],y,color,selected?245:210,clear);
    }
    if(selected)for(int a=0,b=count-1;a<count;b=a++)
        cruise_line(im,px[a],py[a],px[b],py[b],1.25f,CRUISE_TEXT,165);
}
#endif

void adas_hud_intersection(uint8_t *im,const struct features *f,const struct detection *d,
    int n,const struct lane lanes[2],const struct adas_status *s,
    const struct intersection_view *v,int debug,const char *timing) {
#if HUD_UI_V2 && !defined(HUD_TEST_RAW_TEXT)
    intersection_app_render=1;intersection_live=v;
    adas_hud_tracked(im,f,d,n,lanes,s,debug,timing,NULL);
    intersection_app_render=0;intersection_live=NULL;
    intersection_arrow_update(&intersection_arrows,v,s);
    const char *names[3]={"红灯等候","绿灯通行","行人监测"};
    for(int i=0;i<3;i++) {
        int active=i==0?s->light==LIGHT_RED:i==1?s->light==LIGHT_GREEN:v->pedestrian_present;
        unsigned color=active?(i==1?CRUISE_INFO:CRUISE_DANGER):CRUISE_MUTED;
        cruise_rounded(im,595+i*212,38,196,48,22,CRUISE_PANEL,190,color,active?220:80);
        cruise_center(im,693+i*212,51,names[i],22,color,255);
    }
    cruise_text(im,320,51,"按键指令演示",18,CRUISE_MUTED,240,270);
    for(int i=0;i<3;i++) {
        int x=650+i*310;
        enum intersection_permission permission=intersection_arrows.permission[i];
        unsigned color=permission==JUNCTION_ALLOWED?0x3cdc78u:
            permission==JUNCTION_BLOCKED?CRUISE_DANGER:CRUISE_WARN;
        int selected=intersection_arrows.command==i+2;
        intersection_arrow_shape(im,x,i-1,color,selected);
        char label[64];
        snprintf(label,sizeof(label),"%s%s",permission==JUNCTION_ALLOWED?"允许":
            permission==JUNCTION_BLOCKED?"禁止":"待确认",i==0?"左转":i==1?"直行":"右转");
        cruise_rounded(im,x-96,972,192,42,12,CRUISE_PANEL,170,color,selected?210:80);
        cruise_center(im,x,983,label,22,color,255);
    }
    if(v->red_alarm||v->pedestrian_alarm) {
        unsigned alpha=((int)(s->phase_time*4)&1)?240:70;
        for(int i=0;i<8;i++) {
            rect(im,i,i,ADAS_W-2*i,1,CRUISE_DANGER,alpha);
            rect(im,i,ADAS_H-1-i,ADAS_W-2*i,1,CRUISE_DANGER,alpha);
            rect(im,i,i,1,ADAS_H-2*i,CRUISE_DANGER,alpha);
            rect(im,ADAS_W-1-i,i,1,ADAS_H-2*i,CRUISE_DANGER,alpha);
        }
    }
#else
    (void)v;
    adas_hud_tracked(im,f,d,n,lanes,s,debug,timing,NULL);
#endif
}
