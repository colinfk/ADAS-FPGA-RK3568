// Two raster line buffers. Packed channel 0 occupies the low byte.
// Window position 0 (top-left) occupies the low DATA_BITS bits.
module cnn_window3x3 #(parameter WIDTH=34, DATA_BITS=8)(
 input clk, rst_n, start,
 input in_valid, output in_ready, input [DATA_BITS-1:0] in_data,
 output reg out_valid, input out_ready,
 output reg [9*DATA_BITS-1:0] out_window
);
reg [DATA_BITS-1:0] row0[0:WIDTH-1], row1[0:WIDTH-1];
reg [DATA_BITS-1:0] t0,t1,m0,m1,b0,b1;
reg [7:0] x,y;
assign in_ready=(!out_valid || out_ready) && !start;
always @(posedge clk) begin
 if(!rst_n || start) begin
  x<=0; y<=0; t0<=0;t1<=0;m0<=0;m1<=0;b0<=0;b1<=0;
  out_valid<=0;out_window<=0;
 end else begin
  if(out_valid && out_ready) out_valid<=0;
  if(in_valid && in_ready) begin
   row0[x]<=row1[x]; row1[x]<=in_data;
   t0<=t1;t1<=row0[x];m0<=m1;m1<=row1[x];b0<=b1;b1<=in_data;
   if(y>=2 && x>=2) begin
    out_window<={in_data,b1,b0,row1[x],m1,m0,row0[x],t1,t0};
    out_valid<=1;
   end
   if(x==WIDTH-1) begin x<=0;y<=y+1'b1;end
   else x<=x+1'b1;
  end
 end
end
endmodule

