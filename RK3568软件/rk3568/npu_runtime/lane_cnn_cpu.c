#include "lane_cnn_cpu.h"
#include "lane_roi_config.h"
#include <limits.h>
#include <math.h>
#include <string.h>
#include "lane_cnn_params_v2.h"
static unsigned gray565(const uint8_t *p) {
    unsigned word=(unsigned)p[0]|((unsigned)p[1]<<8);
    unsigned r=(word>>11)&31,g=(word>>5)&63,b=word&31;
    r=(r<<3)|(r>>2); g=(g<<2)|(g>>4); b=(b<<3)|(b>>2);
    return (r+2*g+b)>>2;
}
int lane_cnn_guided_top(const struct lane *fit) {
    if(!fit||!fit->valid||!isfinite(fit->slope)||!isfinite(fit->intercept))return -1;
    float horizon=fabsf(fit->slope)>.1f?(960.f-fit->intercept)/fit->slope:0.f;
    if(!isfinite(horizon)||horizon>736.f)return -1;
    if(horizon<480.f)return 512;
    return (int)ceilf((horizon+32.f)/64.f)*64;
}
int lane_cnn_guided_input(const uint8_t *image,const struct lane *fit,uint8_t gray[64*32]) {
    if(!image||!fit||!fit->valid||!isfinite(fit->slope)||!isfinite(fit->intercept)||
       !isfinite(fit->residual)||fit->residual>10||fabsf(fit->slope)>2.35f) return 0;
    int top=lane_cnn_guided_top(fit);if(top<512||top>768)return 0;
    int rows=(1024-top)/64;
    /* Rectify a 128-pixel band below the fitted center-axis horizon.
     * This is RK CPU preprocessing, not a change to the FPGA fixed ROI ABI. */
    for(int oy=0;oy<64;oy++)for(int ox=0;ox<32;ox++) {
        unsigned sum=0;
        for(int iy=0;iy<rows;iy++) {
            int y=top+oy*rows+iy;
            float center=fit->slope*(float)y+fit->intercept;
            if(center<64.f||center>ADAS_W-64.f)return 0;
            int left=(int)floorf(center+.5f)-64;
            if(left<0||left+128>ADAS_W) return 0;
            for(int ix=0;ix<4;ix++) sum+=gray565(image+(y*ADAS_W+left+ox*4+ix)*2);
        }
        gray[oy*32+ox]=(uint8_t)(sum/(unsigned)(rows*4));
    }
    return 1;
}
int lane_cnn_quad_input(const uint8_t *image,const struct lane *fit,uint8_t gray[64*32]) {
    return lane_cnn_guided_input(image,fit,gray);
}
int lane_cnn_roi_hits_version(const struct lane *fit,int side,unsigned version) {return lane_roi_hits_version(fit,side,version);}
int lane_cnn_roi_aligned_version(const struct lane *fit,int side,unsigned version) {return lane_roi_aligned_version(fit,side,version);}
int lane_cnn_roi_hits(const struct lane *fit,int side) {return lane_roi_hits_version(fit,side,4);}
int lane_cnn_roi_aligned(const struct lane *fit,int side) {return lane_roi_aligned_version(fit,side,4);}
int lane_cnn_roi4_input(const uint8_t *image,int side,uint8_t gray[64*32]) {
    if(!image||!gray||side<0||side>1)return 0;
    for(int row=0;row<64;row++) {
        int y=516+8*row,left=perspective_row_x0(side,y);
        for(int col=0;col<32;col++) {
            unsigned sum=2;
            for(int dx=0;dx<4;dx++)sum+=gray565(image+(y*ADAS_W+left+4*col+dx)*2);
            gray[row*32+col]=(uint8_t)(sum>>2);
        }
    }
    return 1;
}
static int conv(const int8_t *a,int8_t *b,int h,int w,int ci,int co,
                const int8_t *weight,const int32_t *bias,unsigned shift) {
    for(int y=0;y<h;y++)for(int x=0;x<w;x++)for(int out=0;out<co;out++) {
        int64_t acc=bias[out];
        for(int in=0;in<ci;in++)for(int ky=0;ky<3;ky++)for(int kx=0;kx<3;kx++) {
            int yy=y+ky-1,xx=x+kx-1;
            if(yy>=0&&yy<h&&xx>=0&&xx<w)
                acc+=(int)a[(yy*w+xx)*ci+in]*(int)weight[((out*ci+in)*3+ky)*3+kx];
        }
        if(acc<INT32_MIN||acc>INT32_MAX) return 0;
        acc=acc<0?0:acc; acc=(acc+(INT64_C(1)<<(shift-1)))>>shift;
        b[(y*w+x)*co+out]=(int8_t)(acc>127?127:acc);
    }
    return 1;
}
static void pool(const int8_t *a,int8_t *b,int h,int w,int c) {
    for(int y=0;y<h/2;y++)for(int x=0;x<w/2;x++)for(int ch=0;ch<c;ch++) {
        int value=0;
        for(int dy=0;dy<2;dy++)for(int dx=0;dx<2;dx++) {
            int v=a[(((y*2+dy)*w+x*2+dx)*c)+ch]; if(v>value)value=v;
        }
        b[(y*(w/2)+x)*c+ch]=(int8_t)value;
    }
}
void lane_cnn_cpu_infer(const uint8_t gray[64*32],struct lane_cpu_result *out) {
    int8_t a[8192],b[8192],bands[32]; memset(out,0,sizeof(*out));out->cls=3;
    for(int i=0;i<2048;i++)a[i]=(int8_t)(gray[i]>>1);
    if(!conv(a,b,64,32,1,4,conv1_w,conv1_b,8))return;
    pool(b,a,64,32,4);
    if(!conv(a,b,32,16,4,8,conv2_w,conv2_b,6))return;
    pool(b,a,32,16,8);
    if(!conv(a,b,16,8,8,8,conv3_w,conv3_b,8))return;
    for(int ch=0;ch<8;ch++)for(int band=0;band<4;band++) {
        int sum=0;
        for(int y=band*4;y<(band+1)*4;y++)for(int x=0;x<8;x++)sum+=b[(y*8+x)*8+ch];
        bands[ch*4+band]=(int8_t)((sum+16)/32);
    }
    for(int cls=0;cls<3;cls++) {
        int64_t acc=fc_b[cls];
        for(int i=0;i<32;i++)acc+=(int)fc_w[cls*32+i]*(int)bands[i];
        if(acc<INT32_MIN||acc>INT32_MAX)return;
        out->logits[cls]=(int32_t)acc;
    }
    unsigned best=0;
    for(unsigned i=1;i<3;i++)if(out->logits[i]>out->logits[best])best=i;
    int32_t second=INT32_MIN;
    for(unsigned i=0;i<3;i++)if(i!=best&&out->logits[i]>second)second=out->logits[i];
    int64_t margin=(int64_t)out->logits[best]-second;
    out->margin=margin>65535?65535:(unsigned)margin;out->valid=margin>=1;out->cls=out->valid?best:3;
}
