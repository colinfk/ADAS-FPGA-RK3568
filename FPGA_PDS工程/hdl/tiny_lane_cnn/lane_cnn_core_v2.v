// Trained lane v2: 1->4->8->8, Pool1/Pool2, four longitudinal means, FC32->3.
module lane_cnn_core_v2 #(
 parameter WEIGHT_FILE="weights_pair.hex",BIAS_FILE="bias_pair.hex",
 parameter CONV1_SHIFT=8,CONV2_SHIFT=6,CONV3_SHIFT=8,REJECT_MARGIN=1,TIMEOUT_CYCLES=3000000
)(
 input clk,rst_n,
 input start,output start_ready,input [31:0] source_frame_id,input model_valid,
 input pixel_valid,output pixel_ready,input [7:0] gray_pixel,
 output result_valid,input result_ready,output reg [31:0] result_frame_id,
 output reg [1:0] class_id,output reg [15:0] confidence,output reg [31:0] margin,
 output reg input_fault,output busy
);
localparam IDLE=0,LOAD=1,C1_START=2,C1=3,C2_START=4,C2=5,C3_START=6,C3=7,FC_REQ=8,FC_WAIT=9,RESULT=10,ABORT=11;
reg [3:0] state;
reg [10:0] load_count;
reg [9:0] position;
reg [31:0] timer;
reg model_q;
reg [7:0] roi[0:2047];
reg [31:0] feature[0:511];
reg [63:0] feature2[0:127];
wire [10:0] roi_addr;
wire [8:0] feature_addr;
wire [6:0] feature2_addr;
reg [7:0] roi_data;
reg [31:0] feature_data;
reg [63:0] feature2_data;
wire r1v,r1r,r2v,r2r,r3v,r3r;
wire reader1_done,reader2_done,reader3_done;
wire [7:0] r1d;
wire [31:0] r2d;
wire [63:0] r3d;
wire w1v,w1r,w2v,w2r,w3v,w3r;
wire [71:0] w1;
wire [287:0] w2;
wire [575:0] w3;
wire engine_ready,engine_valid,pool_ready,pool_valid,pool2_ready,pool2_valid;
wire [63:0] engine_data,pool2_data;
wire [95:0] logits;
wire [31:0] pool_data;
reg [11:0] gap_sum[0:31];
reg [255:0] gap_data;
wire engine_reset_n=rst_n && state!=ABORT;
wire engine_in_valid=(state==C1 && w1v)||(state==C2 && w2v)||(state==C3 && w3v)||(state==FC_REQ);
wire [575:0] engine_window=state==C1 ? {504'd0,w1} : state==C2 ? {288'd0,w2} : state==C3 ? w3 : {320'd0,gap_data};
wire [1:0] operation=state==C1 ? 0 : state==C2 ? 1 : state==C3 ? 2 : 3;
wire engine_out_ready=state==C1 ? pool_ready : state==C2 ? pool2_ready : (state==C3 || state==FC_WAIT);
assign w1r=state==C1 && engine_ready;
assign w2r=state==C2 && engine_ready;
assign w3r=state==C3 && engine_ready;
assign start_ready=state==IDLE;
assign pixel_ready=state==LOAD;
assign result_valid=state==RESULT;
assign busy=state!=IDLE && state!=RESULT;
always @(posedge clk) begin
 roi_data<=roi[roi_addr];feature_data<=feature[feature_addr];feature2_data<=feature2[feature2_addr];
 if(pixel_valid && pixel_ready) roi[load_count]<=gray_pixel>>1;
 if(state==C1 && pool_valid) feature[position[8:0]]<=pool_data;
 if(state==C2 && pool2_valid) feature2[position[6:0]]<=pool2_data;
end
cnn_pad_reader #(32,64,8,11) reader1(clk,engine_reset_n,state==C1_START,roi_addr,roi_data,r1v,r1r,r1d,reader1_done);
cnn_window3x3 #(34,8) win1(clk,engine_reset_n,state==C1_START,r1v,r1r,r1d,w1v,w1r,w1);
cnn_pad_reader #(16,32,32,9) reader2(clk,engine_reset_n,state==C2_START,feature_addr,feature_data,r2v,r2r,r2d,reader2_done);
cnn_window3x3 #(18,32) win2(clk,engine_reset_n,state==C2_START,r2v,r2r,r2d,w2v,w2r,w2);
cnn_pad_reader #(8,16,64,7) reader3(clk,engine_reset_n,state==C3_START,feature2_addr,feature2_data,r3v,r3r,r3d,reader3_done);
cnn_window3x3 #(10,64) win3(clk,engine_reset_n,state==C3_START,r3v,r3r,r3d,w3v,w3r,w3);
cnn_serial_engine_v2 #(WEIGHT_FILE,BIAS_FILE,CONV1_SHIFT,CONV2_SHIFT,CONV3_SHIFT) engine(
 .clk(clk),.rst_n(engine_reset_n),.in_valid(engine_in_valid),.in_ready(engine_ready),
 .operation(operation),.in_window(engine_window),.out_valid(engine_valid),.out_ready(engine_out_ready),
 .out_data(engine_data),.out_logits(logits),.debug_acc0(),.debug_acc1(),
 .debug_capture(),.debug_operation(),.debug_group());
cnn_pool2x2 #(32,4) pool(clk,engine_reset_n,state==C1_START,state==C1 && engine_valid,pool_ready,
 engine_data[31:0],pool_valid,1'b1,pool_data);
cnn_pool2x2 #(16,8) pool2(clk,engine_reset_n,state==C2_START,state==C2 && engine_valid,pool2_ready,
 engine_data,pool2_valid,1'b1,pool2_data);
reg signed [31:0] z0,z1,z2,best,second;
reg [1:0] winner;
reg [32:0] difference;
always @* begin
 z0=logits[31:0];z1=logits[63:32];z2=logits[95:64];best=z0;winner=0;
 if(z1>best)begin best=z1;winner=1;end
 if(z2>best)begin best=z2;winner=2;end
 if(winner==0)second=z1>z2?z1:z2;
 else if(winner==1)second=z0>z2?z0:z2;
 else second=z0>z1?z0:z1;
 difference=$signed({best[31],best})-$signed({second[31],second});
end
integer c,index;
always @(posedge clk) begin
 if(!rst_n)begin
  state<=IDLE;load_count<=0;position<=0;timer<=0;model_q<=0;gap_data<=0;
  result_frame_id<=0;class_id<=3;confidence<=0;margin<=0;input_fault<=0;
  for(c=0;c<32;c=c+1)gap_sum[c]<=0;
 end else begin
  if(busy)timer<=timer+1'b1;
  case(state)
   IDLE:if(start)begin state<=LOAD;load_count<=0;timer<=0;result_frame_id<=source_frame_id;model_q<=model_valid;
    class_id<=3;confidence<=0;margin<=0;input_fault<=0;end
   LOAD:if(pixel_valid)begin if(load_count==2047)begin state<=C1_START;position<=0;end else load_count<=load_count+1'b1;end
   C1_START:state<=C1;
   C1:if(pool_valid)begin if(position==511)begin state<=C2_START;position<=0;end else position<=position+1'b1;end
   C2_START:state<=C2;
   C2:if(pool2_valid)begin if(position==127)begin state<=C3_START;position<=0;
     for(c=0;c<32;c=c+1)gap_sum[c]<=0;gap_data<=0;
    end else position<=position+1'b1;end
   C3_START:state<=C3;
   C3:if(engine_valid)begin
    for(c=0;c<8;c=c+1)begin
     index=c*4+position[6:5];
     gap_sum[index]<=gap_sum[index]+engine_data[c*8+:8];
     if(position[4:0]==31)gap_data[index*8+:8]<=(gap_sum[index]+{4'd0,engine_data[c*8+:8]}+12'd16)>>5;
    end
    if(position==127)state<=FC_REQ;else position<=position+1'b1;
   end
   FC_REQ:if(engine_ready)state<=FC_WAIT;
   FC_WAIT:if(engine_valid)begin margin<=difference[31:0];confidence<=difference>65535?16'hffff:difference[15:0];
    class_id<=model_q && difference>=REJECT_MARGIN?winner:2'd3;state<=RESULT;end
   RESULT:if(result_ready)state<=IDLE;
   ABORT:begin state<=RESULT;class_id<=3;confidence<=0;margin<=0;input_fault<=1;end
   default:state<=IDLE;
  endcase
  if(busy && timer>=TIMEOUT_CYCLES-1 && state!=ABORT)state<=ABORT;
 end
end
endmodule
