// Two active-low user keys; 20 ms at the board's 25 MHz clock.
module lane_key_bar #(parameter DEBOUNCE_CYCLES=500000) (
 input sys_clk,sys_rst_n,input [1:0] keys_n,
 input pcie_clk,pcie_rst_n,output bar_valid,input bar_ready,
 output [11:0] bar_addr,output [127:0] bar_data
);
reg [1:0] reset_pipe;
always @(posedge sys_clk or negedge sys_rst_n)
 if(!sys_rst_n)reset_pipe<=0;else reset_pipe<={reset_pipe[0],1'b1};
wire key_rst_n=reset_pipe[1];
reg [1:0] key_meta,key_sync,stable;
reg [18:0] left_ticks,right_ticks;
reg [31:0] left_presses,right_presses;
always @(posedge sys_clk or negedge key_rst_n) begin
 if(!key_rst_n)begin key_meta<=3;key_sync<=3;stable<=3;left_ticks<=0;right_ticks<=0;left_presses<=0;right_presses<=0;end
 else begin
  key_meta<=keys_n;key_sync<=key_meta;
  if(key_sync[0]==stable[0])left_ticks<=0;
  else if(left_ticks==DEBOUNCE_CYCLES-1)begin
   stable[0]<=key_sync[0];left_ticks<=0;if(!key_sync[0])left_presses<=left_presses+1'b1;
  end else left_ticks<=left_ticks+1'b1;
  if(key_sync[1]==stable[1])right_ticks<=0;
  else if(right_ticks==DEBOUNCE_CYCLES-1)begin
   stable[1]<=key_sync[1];right_ticks<=0;if(!key_sync[1])right_presses<=right_presses+1'b1;
  end else right_ticks<=right_ticks+1'b1;
 end
end
wire [65:0] snapshot;
cdc_snapshot #(66) u_key_snapshot(sys_clk,sys_rst_n,{~stable,right_presses,left_presses},pcie_clk,pcie_rst_n,snapshot);
reg initialized;reg [1:0] phase;reg [31:0] sequence;
reg [65:0] committed,pending;
always @(posedge pcie_clk or negedge pcie_rst_n) begin
 if(!pcie_rst_n)begin initialized<=0;phase<=0;sequence<=0;committed<=0;pending<=0;end
 else if(phase==0)begin
  if(!initialized||snapshot!=committed)begin pending<=snapshot;phase<=1;end
 end else if(bar_ready)begin
  if(phase==3)begin sequence<=sequence+2;committed<=pending;initialized<=1;phase<=0;end
  else phase<=phase+1'b1;
 end
end
// Odd -> new payload while odd -> even: CPU readers reject partial publications.
wire [65:0] payload=phase==1?committed:pending;
wire [31:0] seq_word=sequence+(phase==3?32'd2:32'd1);
assign bar_valid=phase!=0;
assign bar_addr=12'hfeb;
assign bar_data={seq_word,payload[63:32],payload[31:0],payload[65:64],30'h3159454b};
endmodule

// Hold the selected producer through backpressure; CNN bursts retain priority.
module lane_status_mux (
 input clk,rst_n,input cnn_valid,output cnn_ready,input [11:0] cnn_addr,input [127:0] cnn_data,
 input key_valid,output key_ready,input [11:0] key_addr,input [127:0] key_data,
 output valid,input ready,output [11:0] addr,output [127:0] data
);
reg locked,key_grant;
wire select_key=locked?key_grant:(!cnn_valid&&key_valid);
assign valid=select_key?key_valid:cnn_valid;
assign addr=select_key?key_addr:cnn_addr;
assign data=select_key?key_data:cnn_data;
assign cnn_ready=ready&&!select_key;
assign key_ready=ready&&select_key;
always @(posedge clk or negedge rst_n)
 if(!rst_n)begin locked<=0;key_grant<=0;end
 else if(valid)begin
  if(ready)locked<=0;
  else begin locked<=1;key_grant<=select_key;end
 end
endmodule
