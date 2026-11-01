#define _POSIX_C_SOURCE 200809L
#include "preprocess.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
static double seconds(clockid_t clock_id) { struct timespec t; clock_gettime(clock_id,&t); return t.tv_sec+t.tv_nsec/1e9; }
static FILE *create(const char *prefix,const char *suffix) {
    char path[4096]; int n=snprintf(path,sizeof(path),"%s%s",prefix,suffix);
    if(n<0||(size_t)n>=sizeof(path)) return NULL;
    FILE *f=fopen(path,"wbx"); if(!f) perror(path); return f;
}
int main(int argc,char **argv) {
    if(argc<2||argc>5) { fprintf(stderr,"Usage: %s OUTPUT_PREFIX [RGB565_FRAME|-] [RGA_LIBRARY] [DMA_HEAP]\n",argv[0]); return 2; }
    struct preprocessor p; int result=1;
    const char *library=argc>3?argv[3]:"vendor/librga/librga.so";
    uint8_t *reference=malloc(640*640*3); if(!reference) return 1;
    if(prep_open(&p,1,library,argc>4?argv[4]:NULL)) { free(reference); return 1; }
    if(prep_cpu_begin(&p,0)) goto done;
    int pattern=argc<3||!strcmp(argv[2],"-");
    if(pattern) {
        for(int y=0;y<ADAS_H;y++) for(int x=0;x<ADAS_W;x++) {
            unsigned value=x<640?0xf800:x<1280?0x07e0:0x001f;
            size_t at=((size_t)y*ADAS_W+x)*2; p.source[0].data[at]=value&255; p.source[0].data[at+1]=value>>8;
        }
    } else {
        FILE *f=fopen(argv[2],"rb"); if(!f) { perror("input"); goto done; }
        size_t n=fread(p.source[0].data,1,IMAGE_BYTES,f); fclose(f);
        if(n!=IMAGE_BYTES) { fprintf(stderr,"Expected one complete 1920x1080 RGB565 frame\n"); goto done; }
    }
    double start=seconds(CLOCK_MONOTONIC),cpu_start=seconds(CLOCK_THREAD_CPUTIME_ID);
    adas_preprocess(p.source[0].data,reference);
    double cpu_thread_ms=(seconds(CLOCK_THREAD_CPUTIME_ID)-cpu_start)*1000,cpu_wall_ms=(seconds(CLOCK_MONOTONIC)-start)*1000;
    if(prep_cpu_end(&p,0)) goto done;
    start=seconds(CLOCK_MONOTONIC); cpu_start=seconds(CLOCK_THREAD_CPUTIME_ID);
    if(prep_run(&p,0)) goto done;
    double rga_thread_ms=(seconds(CLOCK_THREAD_CPUTIME_ID)-cpu_start)*1000,rga_wall_ms=(seconds(CLOCK_MONOTONIC)-start)*1000;
    unsigned maximum=0,different=0,over8=0; double sum=0; int padding=1,channels=1;
    for(size_t i=0;i<640*640*3;i++) {
        unsigned difference=(unsigned)abs((int)p.output.data[i]-reference[i]);
        if(difference>maximum) maximum=difference;
        different+=difference!=0; over8+=difference>8; sum+=difference;
        unsigned y=(unsigned)(i/(640*3)); if((y<140||y>=500)&&p.output.data[i]!=114) padding=0;
    }
    if(pattern) for(int k=0;k<3;k++) for(int channel=0;channel<3;channel++) {
        size_t i=((size_t)320*640+(106+213*k))*3+(unsigned)channel;
        if(abs((int)p.output.data[i]-reference[i])>8) channels=0;
    }
    FILE *cpu=create(argv[1],"_cpu.rgb"),*rga=create(argv[1],"_rga.rgb"),*metrics=create(argv[1],"_metrics.json");
    int written=cpu&&rga&&metrics;
    if(cpu&&fwrite(reference,1,640*640*3,cpu)!=640*640*3) written=0;
    if(rga&&fwrite(p.output.data,1,640*640*3,rga)!=640*640*3) written=0;
    if(metrics) fprintf(metrics,"{\"pattern\":%s,\"padding_exact\":%s,\"color_bars_checked\":%s,\"color_bars_ok\":%s,\"max_abs_difference\":%u,\"mean_abs_difference\":%.8f,\"different_values\":%u,\"values_over_8\":%u,\"cpu_wall_ms\":%.4f,\"cpu_thread_ms\":%.4f,\"rga_wall_ms\":%.4f,\"rga_thread_ms\":%.4f}\n",
        pattern?"true":"false",padding?"true":"false",pattern?"true":"false",channels?"true":"false",maximum,sum/(640*640*3),different,over8,cpu_wall_ms,cpu_thread_ms,rga_wall_ms,rga_thread_ms);
    if(cpu&&fclose(cpu)) written=0;
    if(rga&&fclose(rga)) written=0;
    if(metrics&&fclose(metrics)) written=0;
    if(prep_output_end(&p)) goto done;
    printf("PREPROCESS_CHECK padding=%d color_bars=%s max_diff=%u mean_diff=%.6f cpu_wall_ms=%.3f rga_wall_ms=%.3f\n",
        padding,pattern?(channels?"PASS":"FAIL"):"not_checked",maximum,sum/(640*640*3),cpu_wall_ms,rga_wall_ms);
    result=written&&padding&&channels?0:1;
done:
    prep_close(&p); free(reference); return result;
}
