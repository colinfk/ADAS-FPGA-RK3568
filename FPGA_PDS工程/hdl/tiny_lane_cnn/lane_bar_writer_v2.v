// PCIe-clock publisher. Four accepted 128-bit writes implement a seqlock.
// Payload must already cross clock domains as one held snapshot.
module lane_bar_writer_v2 # (parameter MODEL_VERSION=0)(
 input clk,rst_n,input in_valid,output in_ready,
 input [31:0] frame_id,flags,config_version,drop_count,
 input [1:0] left_class,right_class,input left_valid,right_valid,
 input [15:0] left_confidence,right_confidence,
 input [31:0] left_margin,right_margin,
 output wr_valid,input wr_ready,output reg [11:0] wr_addr,
 output reg [127:0] wr_data
);
reg [2:0] state;
localparam [15:0] VERSION=MODEL_VERSION;
reg [31:0] sequence_number;
reg [127:0] page0,page1,page2;
assign in_ready=state==0;
assign wr_valid=state!=0;
always @* begin
 wr_addr=12'hfec;wr_data=page0;
 case(state)
  1: wr_data={page0[127:96],sequence_number+32'd1,page0[63:0]};
  2: begin wr_addr=12'hfed;wr_data=page1;end
  3: begin wr_addr=12'hfee;wr_data={page2[127:96],sequence_number+32'd2,page2[63:0]};end
  4: wr_data={page0[127:96],sequence_number+32'd2,page0[63:0]};
  default: begin end
 endcase
end
always @(posedge clk) begin
 if(!rst_n) begin state<=0;sequence_number<=0;page0<=0;page1<=0;page2<=0;end
 else if(state==0) begin
  if(in_valid) begin
   page0<={flags,32'd0,VERSION,16'd1,32'h314e414c};
   page1<={config_version,right_confidence,left_confidence,26'd0,right_valid,left_valid,right_class,left_class,frame_id};
   page2<={drop_count,32'd0,right_margin,left_margin};state<=1;
  end
 end else if(wr_ready) begin
  if(state==4) begin state<=0;sequence_number<=sequence_number+32'd2;end
  else state<=state+1'b1;
 end
end
endmodule
