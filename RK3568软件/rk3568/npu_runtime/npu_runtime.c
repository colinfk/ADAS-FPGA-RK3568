#define _POSIX_C_SOURCE 200809L
#include "npu_runtime.h"
#include <dlfcn.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
static double milliseconds(void) {
    struct timespec t; clock_gettime(CLOCK_MONOTONIC,&t); return t.tv_sec*1000.+t.tv_nsec/1e6;
}
int npu_release(struct npu_runtime *n) {
    if(!n->acquired) return 0;
    int ret=n->outputs_release(n->ctx,9,n->outputs);
    n->acquired=0;
    if(ret<0) fprintf(stderr,"rknn_outputs_release failed: %d\n",ret);
    return ret;
}
void npu_close(struct npu_runtime *n) {
    if(n->initialized) { npu_release(n); n->destroy(n->ctx); }
    if(n->library) dlclose(n->library);
    memset(n,0,sizeof(*n));
}
int npu_open(struct npu_runtime *n,const char *model,const char *library,int model10) {
    memset(n,0,sizeof(*n));
    n->library=dlopen(library,RTLD_NOW|RTLD_LOCAL);
    if(!n->library) { fprintf(stderr,"dlopen %s: %s\n",library,dlerror()); return -1; }
#define LOAD(member,symbol) do { *(void **)(&n->member)=dlsym(n->library,symbol); if(!n->member) { fprintf(stderr,"Missing %s\n",symbol); goto fail; } } while(0)
    LOAD(init,"rknn_init"); LOAD(destroy,"rknn_destroy"); LOAD(query,"rknn_query");
    LOAD(inputs_set,"rknn_inputs_set"); LOAD(run,"rknn_run");
    LOAD(outputs_get,"rknn_outputs_get"); LOAD(outputs_release,"rknn_outputs_release");
#undef LOAD
    FILE *file=fopen(model,"rb");
    if(!file) { perror("model"); goto fail; }
    if(fseek(file,0,SEEK_END)) { fclose(file); goto fail; }
    long length=ftell(file);
    if(length<=0||length>256*1024*1024||fseek(file,0,SEEK_SET)) { fclose(file); goto fail; }
    void *bytes=malloc((size_t)length);
    if(!bytes) { fclose(file); goto fail; }
    size_t read=fread(bytes,1,(size_t)length,file); fclose(file);
    int ret=read==(size_t)length?n->init(&n->ctx,bytes,(uint32_t)length,0,NULL):-1;
    free(bytes);
    if(ret<0) { fprintf(stderr,"rknn_init failed %d: check RK3568 model/runtime/driver\n",ret); goto fail; }
    n->initialized=1;
    rknn_sdk_version version={0};
    if(n->query(n->ctx,RKNN_QUERY_SDK_VERSION,&version,sizeof(version))<0) goto fail;
    fprintf(stderr,"RKNN runtime=%s driver=%s (model built with Toolkit2 2.3.2)\n",version.api_version,version.drv_version);
    rknn_input_output_num num={0};
    if(n->query(n->ctx,RKNN_QUERY_IN_OUT_NUM,&num,sizeof(num))<0||num.n_input!=1||num.n_output!=9) {
        fprintf(stderr,"Expected ONE input and NINE outputs; use best_rk3568_int8_rkopt.rknn\n"); goto fail;
    }
    rknn_tensor_attr input={.index=0};
    if(n->query(n->ctx,RKNN_QUERY_INPUT_ATTR,&input,sizeof(input))<0||input.n_dims!=4||input.dims[0]!=1||
       !((input.fmt==RKNN_TENSOR_NCHW&&input.dims[1]==3&&input.dims[2]==640&&input.dims[3]==640)||
         (input.fmt==RKNN_TENSOR_NHWC&&input.dims[1]==640&&input.dims[2]==640&&input.dims[3]==3))) {
        fprintf(stderr,"Model input must be 1x3x640x640 or 1x640x640x3\n"); goto fail;
    }
    for(unsigned i=0;i<9;i++) {
        rknn_tensor_attr a={.index=i}; unsigned w=80u>>(i/3),c=i%3==0?64:i%3==1?(model10?10:6):1;
        if(n->query(n->ctx,RKNN_QUERY_OUTPUT_ATTR,&a,sizeof(a))<0||a.fmt!=RKNN_TENSOR_NCHW||
           a.n_dims!=4||a.dims[0]!=1||a.dims[1]!=c||a.dims[2]!=w||a.dims[3]!=w||a.n_elems!=c*w*w) {
            fprintf(stderr,"MODEL_OUTPUT_MISMATCH: --model10=%d output %u must be NCHW [1,%u,%u,%u] reg/cls/sum; check model path and mode\n",model10,i,c,w,w); goto fail;
        }
        n->elements[i]=a.n_elems;
        fprintf(stderr,"output=%u shape=1,%u,%u,%u type=%s zp=%d scale=%.9g\n",i,c,w,w,get_type_string(a.type),a.zp,a.scale);
    }
    return 0;
fail:
    npu_close(n); return -1;
}
int npu_infer(struct npu_runtime *n,uint8_t *rgb,const float *outputs[9],double *input_ms,double *run_ms,double *output_ms) {
    if(n->acquired) return -1;
    /* Embedded mean=0/std=255 is applied by Runtime: do not divide or cast to INT8 here. */
    rknn_input input={.index=0,.buf=rgb,.size=640*640*3,.pass_through=0,
        .type=RKNN_TENSOR_UINT8,.fmt=RKNN_TENSOR_NHWC};
    double begin=milliseconds();
    int ret=n->inputs_set(n->ctx,1,&input);
    double input_end=milliseconds();
    if(ret<0) { fprintf(stderr,"rknn_inputs_set failed %d\n",ret); return -1; }
    rknn_run_extend run={.non_block=0,.timeout_ms=1000,.fence_fd=-1};
    ret=n->run(n->ctx,&run);
    double run_end=milliseconds();
    if(ret<0) { fprintf(stderr,"rknn_run failed %d\n",ret); return -1; }
    memset(n->outputs,0,sizeof(n->outputs));
    for(unsigned i=0;i<9;i++) { n->outputs[i].index=i; n->outputs[i].want_float=1; }
    rknn_output_extend output={0};
    ret=n->outputs_get(n->ctx,9,n->outputs,&output);
    if(ret<0) { fprintf(stderr,"rknn_outputs_get failed %d\n",ret); return -1; }
    n->acquired=1;
    if(output.frame_id!=run.frame_id) {
        fprintf(stderr,"RKNN run/output frame IDs disagree\n"); npu_release(n); return -1;
    }
    for(int i=0;i<9;i++) {
        if(!n->outputs[i].buf||n->outputs[i].size!=n->elements[i]*sizeof(float)) {
            fprintf(stderr,"Unexpected FP32 output size at %d\n",i); npu_release(n); return -1;
        }
        outputs[i]=n->outputs[i].buf;
    }
    *input_ms=input_end-begin; *run_ms=run_end-input_end; *output_ms=milliseconds()-run_end;
    return 0;
}
