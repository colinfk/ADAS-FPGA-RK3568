// One shared MAC2 for three convolution layers and Strip4 head and the fully-connected layer.
module cnn_serial_engine_v2 #(
 parameter WEIGHT_FILE="weights_pair.hex", BIAS_FILE="bias_pair.hex",
 parameter CONV1_SHIFT=6, CONV2_SHIFT=6, CONV3_SHIFT=8
)(
 input clk,rst_n,
 input in_valid,output in_ready,input [1:0] operation,input [575:0] in_window,
 output out_valid,input out_ready,output reg [63:0] out_data,
 output reg [95:0] out_logits,
 output wire signed [31:0] debug_acc0,debug_acc1,
 output wire debug_capture,output wire [1:0] debug_operation,output wire [2:0] debug_group
);
localparam IDLE=0,SETUP=1,CLEAR=2,READ=3,STEP=4,CAPTURE=5,OUTPUT=6;
reg [2:0] state,group_id;
reg [6:0] tap;
reg [1:0] op;
reg [575:0] window_q;
reg [15:0] weights[0:513];
reg [63:0] biases[0:11];
reg [15:0] weight_q;
reg [63:0] bias_q;
reg [9:0] weight_addr;
reg [3:0] bias_addr;
reg [7:0] activation;
wire [6:0] taps=op==0 ? 9 : (op==1 ? 36 : (op==2 ? 72 : 32));
wire [2:0] groups=(op==1 || op==2) ? 4 : 2;
wire [1:0] enables=(op==3 && group_id==1) ? 2'b01 : 2'b11;
initial begin $readmemh(WEIGHT_FILE,weights);$readmemh(BIAS_FILE,biases);end
always @* begin
 weight_addr=op==0 ? group_id*9+tap : (op==1 ? 18+group_id*36+tap : (op==2 ? 162+group_id*72+tap : 450+group_id*32+tap));
 bias_addr=op==0 ? group_id : (op==1 ? 2+group_id : (op==2 ? 6+group_id : 10+group_id));
 activation=0;
 if(op==0) activation=window_q[tap*8+:8];
 else if(op==1) activation=window_q[((tap%9)*4+(tap/9))*8+:8];
 else if(op==2) activation=window_q[((tap%9)*8+(tap/9))*8+:8];
 else activation=window_q[tap*8+:8];
end
always @(posedge clk) begin weight_q<=weights[weight_addr];bias_q<=biases[bias_addr];end
tiny_mac2 mac(clk,rst_n,state==CLEAR,state==STEP ? enables : 2'b00,
             activation,weight_q[7:0],weight_q[15:8],bias_q[31:0],bias_q[63:32],
             debug_acc0,debug_acc1);
assign in_ready=state==IDLE;
assign out_valid=state==OUTPUT;
assign debug_capture=state==CAPTURE;
assign debug_operation=op;
assign debug_group=group_id;
function [7:0] quant;
 input signed [31:0] value;
 input integer shift;
 reg [32:0] scaled;
 begin
  if(value[31]) quant=0;
  else begin
   scaled={1'b0,value};
   if(shift>0) scaled=(scaled+(33'd1<<(shift-1)))>>shift;
   quant=scaled>127 ? 8'd127 : scaled[7:0];
  end
 end
endfunction
always @(posedge clk) begin
 if(!rst_n) begin state<=IDLE;group_id<=0;tap<=0;op<=0;window_q<=0;out_data<=0;out_logits<=0;end
 else case(state)
  IDLE: if(in_valid) begin
   op<=operation;window_q<=in_window;group_id<=0;tap<=0;out_data<=0;out_logits<=0;state<=SETUP;
  end
  SETUP: state<=CLEAR;
  CLEAR: begin tap<=0;state<=READ;end
  READ: state<=STEP;
  STEP: if(tap==taps-1) state<=CAPTURE;else begin tap<=tap+1'b1;state<=READ;end
  CAPTURE: begin
   if(op==3) begin
    out_logits[group_id*64+:32]<=debug_acc0;
    if(group_id==0) out_logits[32+:32]<=debug_acc1;
   end else begin
    out_data[group_id*16+:8]<=quant(debug_acc0,op==0 ? CONV1_SHIFT : (op==1 ? CONV2_SHIFT : CONV3_SHIFT));
    out_data[group_id*16+8+:8]<=quant(debug_acc1,op==0 ? CONV1_SHIFT : (op==1 ? CONV2_SHIFT : CONV3_SHIFT));
   end
   if(group_id==groups-1) state<=OUTPUT;
   else begin group_id<=group_id+1'b1;tap<=0;state<=SETUP;end
  end
  OUTPUT: if(out_ready) state<=IDLE;
  default: state<=IDLE;
 endcase
end
endmodule

