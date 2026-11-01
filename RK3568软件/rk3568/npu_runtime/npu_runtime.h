#ifndef NPU_RUNTIME_H
#define NPU_RUNTIME_H
#include "vendor/rknn_api.h"
struct npu_runtime {
    void *library;
    rknn_context ctx;
    int initialized,acquired;
    uint32_t elements[9];
    rknn_output outputs[9];
    int (*init)(rknn_context *,void *,uint32_t,uint32_t,rknn_init_extend *);
    int (*destroy)(rknn_context);
    int (*query)(rknn_context,rknn_query_cmd,void *,uint32_t);
    int (*inputs_set)(rknn_context,uint32_t,rknn_input *);
    int (*run)(rknn_context,rknn_run_extend *);
    int (*outputs_get)(rknn_context,uint32_t,rknn_output *,rknn_output_extend *);
    int (*outputs_release)(rknn_context,uint32_t,rknn_output *);
};
int npu_open(struct npu_runtime *n,const char *model,const char *library,int model10);
int npu_infer(struct npu_runtime *n,uint8_t *rgb,const float *outputs[9],double *input_ms,double *run_ms,double *output_ms);
int npu_release(struct npu_runtime *n);
void npu_close(struct npu_runtime *n);
#endif
