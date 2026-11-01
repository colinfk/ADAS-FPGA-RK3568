// Video -> cropped ROI pairs -> shared CNN -> coherent PCIe BAR writes.
// Inference can reuse the existing globally routed HDMI configuration PLL.
module lane_cnn_sidecar_v2 #(
 parameter LEFT_X=480,RIGHT_X=1312,TOP_Y=512,ROI_CONFIG_VERSION=4,
 parameter WEIGHT_FILE="weights_pair.hex",BIAS_FILE="bias_pair.hex",
 parameter CONV1_SHIFT=6,CONV2_SHIFT=6,CONV3_SHIFT=8,REJECT_MARGIN=1,MODEL_VERSION=0,MODEL_VALID=0
)(
 input video_clk,video_rst_n,video_ready,vs,de,input [15:0] rgb565,
 input [31:0] source_frame_id,
 input cnn_clk,cnn_rst_n,pcie_clk,pcie_rst_n,
 output bar_valid,input bar_ready,output [11:0] bar_addr,output [127:0] bar_data
);
wire link_reset_n=video_rst_n && cnn_rst_n && pcie_rst_n;
reg [1:0] vr,cr,pr;
always @(posedge video_clk or negedge link_reset_n)if(!link_reset_n)vr<=0;else vr<={vr[0],1'b1};
always @(posedge cnn_clk or negedge link_reset_n)if(!link_reset_n)cr<=0;else cr<={cr[0],1'b1};
always @(posedge pcie_clk or negedge link_reset_n)if(!link_reset_n)pr<=0;else pr<={pr[0],1'b1};
reg vs_q,de_q,armed;reg [11:0] x,y;reg [31:0] video_frame_id;
wire frame_start=video_ready && vs && !vs_q;
// The shared capture counter is settled by first DE, after enhancement latency.
wire pair_frame_start=armed && video_ready && de && !de_q && x==0 && y==0;
always @(posedge video_clk)begin
 if(!vr[1])begin vs_q<=0;de_q<=0;armed<=0;x<=0;y<=0;video_frame_id<=0;end
 else begin
  vs_q<=vs;de_q<=de;
  if(!video_ready)begin armed<=0;x<=0;y<=0;end
  else if(frame_start)begin armed<=1;x<=0;y<=0;video_frame_id<=video_frame_id+1'b1;end
  else if(armed)begin
   if(de)x<=x+1'b1;
   else if(de_q)begin x<=0;y<=y+1'b1;end
  end
 end
end
wire lv,rv;wire [10:0] la,ra;wire [7:0] lg,rg;
lane_roi_crop #(LEFT_X,RIGHT_X,TOP_Y) crop(video_clk,vr[1],frame_start,armed&&video_ready&&de,x,y,rgb565,lv,rv,la,ra,lg,rg);
wire pair_valid,pair_take,pair_bank,pair_release,side,result_valid,result_fault;
wire [10:0] address;wire [7:0] read_data;
wire [31:0] video_drops,pair_frame,pair_drops,result_frame,result_drops,left_margin,right_margin;
wire [1:0] left_class,right_class;wire [15:0] left_confidence,right_confidence;
lane_pair_buffer buffers(video_clk,vr[1],pair_frame_start,source_frame_id,lv,rv,la,ra,lg,rg,video_drops,
 cnn_clk,cr[1],pair_valid,pair_take,pair_bank,pair_frame,pair_drops,pair_release,side,address,read_data);
lane_pair_scheduler_v2 #(WEIGHT_FILE,BIAS_FILE,CONV1_SHIFT,CONV2_SHIFT,CONV3_SHIFT,REJECT_MARGIN) scheduler(
 cnn_clk,cr[1],MODEL_VALID!=0,pair_valid,pair_take,pair_frame,pair_drops,pair_release,side,address,read_data,
 result_valid,1'b1,result_frame,result_drops,left_class,right_class,left_confidence,right_confidence,left_margin,right_margin,result_fault);
// Keep latest stable completed payload for pull-style CDC. Initial publication
// carries INVALID and ready=0; its frame id is zero.
reg [223:0] latest;
localparam [31:0] CONFIG_VERSION=ROI_CONFIG_VERSION;
wire [31:0] flags={28'd0,(result_drops!=0),result_fault,1'b0,1'b1};
wire [31:0] classes={26'd0,(right_class!=3),(left_class!=3),right_class,left_class};
always @(posedge cnn_clk)begin
 if(!cr[1])latest<={32'd0,32'd0,32'd0,32'd0,32'hf,32'd0,32'd0};
 else if(result_valid)latest<={result_drops,right_margin,left_margin,right_confidence,left_confidence,classes,result_frame,flags};
end
wire [223:0] snapshot;
cdc_snapshot #(224) result_cdc(cnn_clk,cr[1],latest,pcie_clk,pr[1],snapshot);
reg initialized;reg [223:0] previous;
wire publish_ready;
wire publish_valid=!initialized || previous!=snapshot;
always @(posedge pcie_clk)begin
 if(!pr[1])begin initialized<=0;previous<=0;end
 else if(publish_valid && publish_ready)begin initialized<=1;previous<=snapshot;end
end
lane_bar_writer_v2 #(MODEL_VERSION) publisher(pcie_clk,pr[1],publish_valid,publish_ready,
 snapshot[63:32],snapshot[31:0],CONFIG_VERSION,snapshot[223:192],
 (snapshot[0]?snapshot[65:64]:2'd3),(snapshot[0]?snapshot[67:66]:2'd3),snapshot[68],snapshot[69],snapshot[111:96],snapshot[127:112],
 snapshot[159:128],snapshot[191:160],bar_valid,bar_ready,bar_addr,bar_data);
endmodule
