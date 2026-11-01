#ifndef LANE_CNN_V2_H
#define LANE_CNN_V2_H
#include "adas_core.h"
#include <stdint.h>
#define LANE_CNN_BAR_BASE 0xfec0u
struct lane_cnn_snapshot {
    uint32_t sequence,flags,frame_id,roi_version,drops,margin[2];
    unsigned model_version,cls[2],valid[2],confidence[2];
};
struct lane_cnn_reader {
    int fd;
    const volatile uint32_t *map;
    struct lane_cnn_snapshot snapshot;
    double changed;
    int observed,progress;
    struct lane_cnn_snapshot history[4];
    double history_changed[4];
    unsigned history_count,history_next;
};
struct lane_cnn_display {
    enum lane_class cls;
    unsigned hist[3],count,missing;
    uint32_t sequence,frame;
    int initialized,have_sample,held;
    double last_valid;
};
/* Presentation only; a CNN observation never manufactures valid lane geometry. */
void lane_cnn_display_update(struct lane_cnn_display *display,
    const struct lane_cnn_snapshot *snapshot,unsigned side,double now,int reset);
int lane_cnn_open(struct lane_cnn_reader *reader,const char *resource0);
void lane_cnn_close(struct lane_cnn_reader *reader);
int lane_cnn_decode(const uint32_t words[12],struct lane_cnn_snapshot *snapshot);
int lane_cnn_read_words(const volatile uint32_t *map,struct lane_cnn_snapshot *snapshot);
int lane_cnn_poll(struct lane_cnn_reader *reader,double now);
int lane_cnn_fresh(const struct lane_cnn_reader *reader,uint32_t source_id,double now);
const struct lane_cnn_snapshot *lane_cnn_for_frame(const struct lane_cnn_reader *reader,uint32_t source_id,double now);
/* Returns a side mask. Unknown/rejected/unfresh results preserve the rules baseline. */
unsigned lane_cnn_apply(const struct lane_cnn_reader *reader,uint32_t source_id,double now,struct lane lanes[2]);
const char *lane_cnn_label(unsigned cls);
#define LANE_KEY_BAR_BASE 0xfeb0u
struct lane_key_reader {uint32_t sequence,left,right;int initialized;};
/* 1: new request (-1 left, +1 right, 0 cancel), 0: no event, -1: invalid ABI. */
int lane_key_poll(const volatile uint32_t *map,struct lane_key_reader *reader,int *turn);
#endif
