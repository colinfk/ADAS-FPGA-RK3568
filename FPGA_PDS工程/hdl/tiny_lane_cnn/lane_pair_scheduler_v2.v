// One shared core evaluates both sides of an owned bank, then releases bank.
module lane_pair_scheduler_v2 #(
 parameter WEIGHT_FILE="weights_pair.hex",BIAS_FILE="bias_pair.hex",
 parameter CONV1_SHIFT=6,CONV2_SHIFT=6,CONV3_SHIFT=8,REJECT_MARGIN=1
)(
 input clk,rst_n,model_valid,
 input pair_valid,output pair_take,input [31:0] pair_frame_id,pair_drop_count,
 output pair_release,output read_side,output [10:0] read_addr,input [7:0] read_data,
 output result_valid,input result_ready,output reg [31:0] frame_id,drop_count,
 output reg [1:0] left_class,right_class,output reg [15:0] left_confidence,right_confidence,
 output reg [31:0] left_margin,right_margin,output reg input_fault
);
localparam IDLE=0,META=1,START=2,READ=3,LATCH=4,SEND=5,WAIT=6,RESULT=7;
reg [2:0] state;
reg side,model_q;reg [10:0] index;reg [7:0] pixel_q;
wire start_ready,pixel_ready,core_valid,core_fault,core_busy;
wire [31:0] core_frame,core_margin;wire [1:0] core_class;wire [15:0] core_confidence;
assign pair_take=state==IDLE;
assign pair_release=state==WAIT && core_valid && side;
assign read_side=side;assign read_addr=index;assign result_valid=state==RESULT;
lane_cnn_core_v2 #(WEIGHT_FILE,BIAS_FILE,CONV1_SHIFT,CONV2_SHIFT,CONV3_SHIFT,REJECT_MARGIN) core(
 clk,rst_n,state==START,start_ready,frame_id,model_q,state==SEND,pixel_ready,pixel_q,
 core_valid,state==WAIT,core_frame,core_class,core_confidence,core_margin,core_fault,core_busy);
always @(posedge clk)begin
 if(!rst_n)begin
  state<=IDLE;side<=0;index<=0;pixel_q<=0;model_q<=0;frame_id<=0;drop_count<=0;
  left_class<=3;right_class<=3;left_confidence<=0;right_confidence<=0;
  left_margin<=0;right_margin<=0;input_fault<=0;
 end else case(state)
  IDLE:if(pair_valid)begin state<=META;side<=0;model_q<=model_valid;input_fault<=0;end
  META:begin frame_id<=pair_frame_id;drop_count<=pair_drop_count;state<=START;index<=0;end
  START:if(start_ready)state<=READ;
  READ:state<=LATCH;
  LATCH:begin pixel_q<=read_data;state<=SEND;end
  SEND:if(pixel_ready)begin
   if(index==2047)state<=WAIT;else begin index<=index+1'b1;state<=READ;end
  end
  WAIT:if(core_valid)begin
   input_fault<=input_fault|core_fault;
   if(!side)begin
    left_class<=core_class;left_confidence<=core_confidence;left_margin<=core_margin;
    side<=1;index<=0;state<=START;
   end else begin right_class<=core_class;right_confidence<=core_confidence;right_margin<=core_margin;state<=RESULT;end
  end
  RESULT:if(result_ready)state<=IDLE;
  default:state<=IDLE;
 endcase
end
endmodule
