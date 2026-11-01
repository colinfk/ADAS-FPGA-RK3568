#define _GNU_SOURCE
#include "preprocess.h"
#include "vendor/librga/include/im2d.h"
#include "vendor/librga/include/rga.h"
#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <linux/dma-buf.h>
#include <linux/dma-heap.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>
struct rga_api {
    void *library;
    rga_buffer_handle_t (*import_fd)(int,im_handle_param_t *);
    IM_STATUS (*release)(rga_buffer_handle_t);
    rga_buffer_t (*wrap)(rga_buffer_handle_t,int,int,int,int,int);
    IM_STATUS (*check)(rga_buffer_t,rga_buffer_t,rga_buffer_t,im_rect,im_rect,im_rect,int);
    IM_STATUS (*process)(rga_buffer_t,rga_buffer_t,rga_buffer_t,im_rect,im_rect,im_rect,int);
    IM_STATUS (*fill)(rga_buffer_t,im_rect,int,int);
    const char *(*error)(IM_STATUS);
    const char *(*query)(int);
    rga_buffer_t sources[PREP_SOURCE_SLOTS],destination;
};
static int dma_sync(struct prep_buffer *b,int begin,uint64_t direction) {
    struct dma_buf_sync sync={.flags=direction|(begin?DMA_BUF_SYNC_START:DMA_BUF_SYNC_END)};
    int ret; do { ret=ioctl(b->fd,DMA_BUF_IOCTL_SYNC,&sync); } while(ret<0&&errno==EINTR);
    if(ret) { perror("DMA_BUF_IOCTL_SYNC"); return -1; }
    b->active=begin; return 0;
}
int prep_cpu_begin(struct preprocessor *p,int index) {
    if(index<0||index>=PREP_SOURCE_SLOTS) return -1;
    if(!p->rga) return 0;
    if(p->source[index].active) { fprintf(stderr,"RGA source CPU ownership already active\n"); return -1; }
    return dma_sync(&p->source[index],1,DMA_BUF_SYNC_RW);
}
int prep_cpu_end(struct preprocessor *p,int index) {
    if(index<0||index>=PREP_SOURCE_SLOTS) return -1;
    if(!p->rga) return 0;
    if(!p->source[index].active) { fprintf(stderr,"RGA source CPU ownership missing\n"); return -1; }
    return dma_sync(&p->source[index],0,DMA_BUF_SYNC_RW);
}
int prep_output_end(struct preprocessor *p) {
    if(!p->rga) return 0;
    if(!p->output.active) return -1;
    return dma_sync(&p->output,0,DMA_BUF_SYNC_READ);
}
static int allocate(struct preprocessor *p,struct prep_buffer *b,size_t bytes,int w,int h,int format) {
    b->bytes=bytes;
    if(!p->rga) { b->data=malloc(bytes); return b->data?0:-1; }
    struct dma_heap_allocation_data allocation={.len=bytes,.fd_flags=O_RDWR|O_CLOEXEC};
    if(ioctl(p->heap_fd,DMA_HEAP_IOCTL_ALLOC,&allocation)) { perror("DMA_HEAP_IOCTL_ALLOC"); return -1; }
    b->fd=(int)allocation.fd;
    void *address=mmap(NULL,bytes,PROT_READ|PROT_WRITE,MAP_SHARED,b->fd,0);
    if(address==MAP_FAILED) { perror("DMA-BUF mmap"); return -1; }
    b->data=address;
    struct rga_api *api=p->implementation;
    im_handle_param_t param={.width=(uint32_t)w,.height=(uint32_t)h,.format=(uint32_t)format};
    b->handle=api->import_fd(b->fd,&param);
    if(!b->handle) { fprintf(stderr,"RGA importbuffer_fd failed: %s\n",api->error(IM_STATUS_INVALID_PARAM)); return -1; }
    return 0;
}
void prep_close(struct preprocessor *p) {
    struct rga_api *api=p->implementation;
    for(int i=0;i<PREP_SOURCE_SLOTS+1;i++) {
        struct prep_buffer *b=i<PREP_SOURCE_SLOTS?&p->source[i]:&p->output;
        if(p->rga) {
            if(b->active) dma_sync(b,0,i<PREP_SOURCE_SLOTS?DMA_BUF_SYNC_RW:DMA_BUF_SYNC_READ);
            if(b->handle&&api&&api->release&&api->release(b->handle)!=IM_STATUS_SUCCESS) fprintf(stderr,"RGA releasebuffer_handle failed\n");
            if(b->data) munmap(b->data,b->bytes);
            if(b->fd>=0) close(b->fd);
        } else free(b->data);
        memset(b,0,sizeof(*b)); b->fd=-1;
    }
    if(p->heap_fd>=0) close(p->heap_fd);
    if(api) { if(api->library) dlclose(api->library); free(api); }
    p->implementation=NULL; p->heap_fd=-1;
}
int prep_open(struct preprocessor *p,int rga,const char *library,const char *heap) {
    memset(p,0,sizeof(*p)); p->rga=rga; p->heap_fd=-1; p->output.fd=-1;
    for(int i=0;i<PREP_SOURCE_SLOTS;i++) p->source[i].fd=-1;
    if(rga) {
        struct rga_api *api=calloc(1,sizeof(*api)); if(!api) goto failed;
        p->implementation=api; api->library=dlopen(library,RTLD_NOW|RTLD_LOCAL);
        if(!api->library) { fprintf(stderr,"RGA dlopen %s: %s\n",library,dlerror()); goto failed; }
#define LOAD(member,name) do { *(void **)(&api->member)=dlsym(api->library,name); if(!api->member) { fprintf(stderr,"RGA C symbol missing: %s\n",name); goto failed; } } while(0)
        LOAD(import_fd,"importbuffer_fd"); LOAD(release,"releasebuffer_handle"); LOAD(wrap,"wrapbuffer_handle_t");
        LOAD(check,"imcheck_t"); LOAD(process,"improcess"); LOAD(fill,"imfill_t");
        LOAD(error,"imStrError_t"); LOAD(query,"querystring");
#undef LOAD
        const char *info=api->query(RGA_ALL);
        fprintf(stderr,"PREPROCESS backend=rga library=%s\n%s\n",library,info?info:"RGA capability query unavailable");
        /* CPU analysis/HUD also access source buffers; prefer cached DMA heaps. */
        const char *paths[]={"/dev/dma_heap/system-dma32","/dev/dma_heap/system",
            "/dev/dma_heap/dma32-uncached","/dev/dma_heap/system-uncached-dma32","/dev/dma_heap/system-uncached",
            "/dev/dma_heap/cma","/dev/dma_heap/linux,cma",NULL};
        const char *selected=heap;
        if(heap) p->heap_fd=open(heap,O_RDWR|O_CLOEXEC);
        else for(int i=0;paths[i];i++) { p->heap_fd=open(paths[i],O_RDWR|O_CLOEXEC); if(p->heap_fd>=0) { selected=paths[i]; break; } }
        if(p->heap_fd<0) { perror("RGA DMA heap open (use --rga-heap PATH)"); goto failed; }
        fprintf(stderr,"RGA DMA heap=%s\n",selected);
    } else fprintf(stderr,"PREPROCESS backend=cpu\n");
    /* FPGA little-endian words store red in bits 15:11; RGA calls this BGR565. */
    for(int i=0;i<PREP_SOURCE_SLOTS;i++) if(allocate(p,&p->source[i],rga?ALLOC_BYTES:PAYLOAD_BYTES,ADAS_W,ADAS_H,RK_FORMAT_BGR_565)) goto failed;
    if(allocate(p,&p->output,640*640*3,640,640,RK_FORMAT_RGB_888)) goto failed;
    if(rga) {
        struct rga_api *api=p->implementation;
        for(int i=0;i<PREP_SOURCE_SLOTS;i++) api->sources[i]=api->wrap(p->source[i].handle,ADAS_W,ADAS_H,ADAS_W,ADAS_H,RK_FORMAT_BGR_565);
        api->destination=api->wrap(p->output.handle,640,640,640,640,RK_FORMAT_RGB_888);
        im_rect source={0,0,ADAS_W,ADAS_H},destination={0,140,640,360},empty={0}; rga_buffer_t unused={0};
        for(int i=0;i<PREP_SOURCE_SLOTS;i++) {
            IM_STATUS status=api->check(api->sources[i],api->destination,unused,source,destination,empty,IM_SYNC);
            if(status!=IM_STATUS_NOERROR&&status!=IM_STATUS_SUCCESS) { fprintf(stderr,"RGA imcheck: %s (%d)\n",api->error(status),status); goto failed; }
        }
        /* Initialize padding once; every synchronous job overwrites only the inner rectangle. */
        IM_STATUS status=api->fill(api->destination,(im_rect){0,0,640,640},0xff727272,1);
        if(status!=IM_STATUS_SUCCESS) { fprintf(stderr,"RGA padding fill: %s (%d)\n",api->error(status),status); goto failed; }
    }
    return 0;
failed:
    prep_close(p); return -1;
}
int prep_run(struct preprocessor *p,int index) {
    if(index<0||index>=PREP_SOURCE_SLOTS) return -1;
    if(!p->rga) { adas_preprocess(p->source[index].data,p->output.data); return 0; }
    if(p->source[index].active||p->output.active) { fprintf(stderr,"RGA job submitted during CPU access\n"); return -1; }
    struct rga_api *api=p->implementation; rga_buffer_t unused={0}; im_rect empty={0};
    IM_STATUS status=api->process(api->sources[index],api->destination,unused,
        (im_rect){0,0,ADAS_W,ADAS_H},(im_rect){0,140,640,360},empty,IM_SYNC);
    if(status!=IM_STATUS_SUCCESS) { fprintf(stderr,"RGA resize/RGB565->RGB888: %s (%d)\n",api->error(status),status); return -1; }
    /* IM_SYNC completes before CPU/NPU input APIs are allowed to read the RGB buffer. */
    return dma_sync(&p->output,1,DMA_BUF_SYNC_READ);
}
