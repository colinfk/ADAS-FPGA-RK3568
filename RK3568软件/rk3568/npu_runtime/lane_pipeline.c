#include "lane_pipeline.h"
#include "lane_cnn_cpu.h"
#include <math.h>
#include <string.h>
void lane_pipeline_step(struct lane_pipeline *p,const uint8_t *image,
    const struct features *f,const struct detection *d,int n,int mode,
    const struct lane_cnn_snapshot *cnn,double now,int reset,struct lane_observation *out) {
    int quad=(f->source_kind&ADAS_SOURCE_QUAD)!=0;
    memset(out,0,sizeof(*out));
    for(unsigned side=0;side<2;side++) {
        out->raw_class[side]=LANE_CLASS_INVALID;
        struct lane fit;
        if(quad)adas_lane_quad_masked(image,(int)side,d,n,&fit);
        else adas_lane_masked(image,&f->candidates[side],(int)side,d,n,&fit);
        if(!quad&&(f->source_kind&ADAS_SOURCE_STABLE_HUD))adas_lane_reject_hood(image,d,n,&fit);
        out->fit[side]=fit;out->roi_hits[side]=lane_cnn_roi_hits(&fit,(int)side);
        struct lane classified=fit;
        struct lane_cnn_snapshot cpu={0};
        struct lane_cnn_snapshot corrected;
        const struct lane_cnn_snapshot *sample=cnn;
        enum lane_source source=mode?LANE_SOURCE_WAIT:LANE_SOURCE_RULES;
        if(mode) {
            classified.type=LANE_UNKNOWN;
            if(cnn&&(!quad||cnn->roi_version==4)&&cnn->valid[side]&&cnn->cls[side]<3&&
               (mode==1||lane_cnn_roi_aligned_version(&fit,(int)side,cnn->roi_version))) source=LANE_SOURCE_FPGA;
            else if(mode==2) {
                uint8_t gray[64*32];struct lane_cpu_result result;
                sample=NULL;
                if(quad?lane_cnn_quad_input(image,&fit,gray):lane_cnn_guided_input(image,&fit,gray)) {
                    out->top[side]=quad?QUAD_ROAD_MIN:lane_cnn_guided_top(&fit);
                    lane_cnn_cpu_infer(gray,&result);
                    cpu.sequence=f->source_id;cpu.frame_id=f->source_id;
                    cpu.model_version=2;cpu.roi_version=3;
                    cpu.valid[side]=result.valid;cpu.cls[side]=result.cls;
                    cpu.confidence[side]=result.margin;
                    sample=&cpu;source=LANE_SOURCE_CPU_ROI;out->guided[side]=1;
                }
            }
            if(sample&&source!=LANE_SOURCE_WAIT&&sample->valid[side]) {
                unsigned cls=sample->cls[side];
                out->raw_class[side]=cls;out->raw_confidence[side]=sample->confidence[side];
                classified.type=adas_lane_type((enum lane_class)cls);
                if((mode==1||mode==2)&&classified.type==LANE_SOLID&&fit.valid&&
                   fit.type==LANE_DASHED&&fit.coverage>=.20f&&fit.coverage<=.60f&&
                   sample->confidence[side]<(source==LANE_SOURCE_FPGA?2400:1900)) {
                    classified.type=LANE_DASHED;
                    out->gap_veto[side]=1;
                }
                if(fit.valid&&fit.type==LANE_SOLID&&fit.coverage>=.64f&&
                   (classified.type==LANE_UNKNOWN||
                    (classified.type==LANE_DASHED&&(sample->confidence[side]<1800||
                     (quad&&fit.coverage>=.85f&&fit.residual<=3.f))))) {
                    classified.type=LANE_SOLID;out->continuity_guard[side]=1;
                }
                if(fit.valid&&fit.type==LANE_DASHED&&fit.coverage>=.22f&&classified.type==LANE_UNKNOWN)
                    classified.type=LANE_DASHED;
                corrected=*sample;
                corrected.cls[side]=classified.type==LANE_DASHED?1:classified.type==LANE_SOLID?2:0;
                sample=&corrected;
                if(fit.valid&&classified.type!=LANE_UNKNOWN)out->applied|=1u<<side;
            }
        }
        int source_reset=reset||(source!=LANE_SOURCE_WAIT&&source!=p->previous_source[side]);
        if(source!=LANE_SOURCE_WAIT)p->previous_source[side]=source;
        out->source[side]=source;
        adas_lane_stabilize(&p->classes[side],&classified,source_reset);
        struct lane geometry=fit;
        adas_lane_geometry_stabilize(&p->geometry[side],&geometry,reset);
        /* Class hold is presentation-only. Current confirmed geometry can exist
         * with UNKNOWN type, and cannot inherit a held SOLID crossing decision. */
        geometry.type=classified.valid?classified.type:LANE_UNKNOWN;
        out->lanes[side]=geometry;
        if(mode)lane_cnn_display_update(&p->display[side],sample,side,now,source_reset);
    }
}
void lane_pipeline_status(const struct lane_pipeline *p,const struct lane_observation *out,
    const struct lane_cnn_snapshot *cnn,int mode,double now,struct adas_status *status) {
    status->display_cnn=mode!=0;
    status->cnn_model_version=cnn?cnn->model_version:0;
    status->cnn_roi_version=cnn?cnn->roi_version:0;
    for(int side=0;side<2;side++) {
        status->gap_veto[side]=out->gap_veto[side];
        status->lane_source[side]=out->source[side];status->roi_hits[side]=out->roi_hits[side];
        status->roi_guided[side]=out->guided[side];
        status->roi_top[side]=out->top[side];
        status->roi_slope[side]=out->fit[side].slope;status->roi_intercept[side]=out->fit[side].intercept;
        if(mode) {
            const struct lane_cnn_display *v=&p->display[side];
            status->display_class[side]=v->cls;status->display_held[side]=v->held;
            status->display_frame[side]=v->frame;
            status->display_age_ms[side]=v->have_sample?fmax(0,now-v->last_valid)*1000:0;
        } else {
            enum lane_type t=p->classes[side].stable;
            /* Existing rules class hold remains visible; geometry is independent. */
            status->display_class[side]=adas_lane_class(t);
            status->display_held[side]=p->classes[side].hold_miss>0&&t!=LANE_UNKNOWN;
        }
        /* Present the same confirmed/coasting type that HUD draws; never restore geometry. */
        enum lane_type drawn=out->lanes[side].valid?out->lanes[side].type:LANE_UNKNOWN;
        if(drawn==LANE_UNKNOWN&&status->coasting_valid[side]) {
            drawn=status->coasting_type[side];status->display_held[side]=1;
        }
        if(drawn!=LANE_UNKNOWN)
            status->display_class[side]=adas_lane_class(drawn);
    }
}
