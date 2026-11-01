#ifndef ADAS_PREPROCESS_H
#define ADAS_PREPROCESS_H
#include "adas_core.h"
/* Capture, latest input, NPU, completed result, CPU/display each own a slot. */
#define PREP_SOURCE_SLOTS 5
struct prep_buffer { uint8_t *data; size_t bytes; int fd,active; uint32_t handle; };
struct preprocessor {
    int rga,heap_fd;
    void *implementation;
    struct prep_buffer source[PREP_SOURCE_SLOTS],output;
};
int prep_open(struct preprocessor *p,int rga,const char *library,const char *heap);
int prep_cpu_begin(struct preprocessor *p,int index);
int prep_cpu_end(struct preprocessor *p,int index);
int prep_run(struct preprocessor *p,int index);
int prep_output_end(struct preprocessor *p);
void prep_close(struct preprocessor *p);
#endif
