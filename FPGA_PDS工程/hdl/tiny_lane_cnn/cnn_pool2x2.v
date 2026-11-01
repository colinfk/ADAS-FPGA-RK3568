module cnn_pool2x2 #(parameter WIDTH=32, CHANNELS=4)(
 input clk,rst_n,start,
 input in_valid,output in_ready,input [CHANNELS*8-1:0] in_data,
 output reg out_valid,input out_ready,output reg [CHANNELS*8-1:0] out_data
);
reg [CHANNELS*8-1:0] line[0:WIDTH-1];
reg [CHANNELS*8-1:0] left,top_left;
reg [7:0] x,y;
integer c;
reg [7:0] maximum;
assign in_ready=(!out_valid || out_ready) && !start;
always @(posedge clk) begin
 if(!rst_n || start) begin x<=0;y<=0;left<=0;top_left<=0;out_valid<=0;out_data<=0;end
 else begin
  if(out_valid && out_ready) out_valid<=0;
  if(in_valid && in_ready) begin
   line[x]<=in_data;left<=in_data;top_left<=line[x];
   if(x[0] && y[0]) begin
    for(c=0;c<CHANNELS;c=c+1) begin
     maximum=in_data[c*8+:8];
     if(left[c*8+:8]>maximum) maximum=left[c*8+:8];
     if(line[x][c*8+:8]>maximum) maximum=line[x][c*8+:8];
     if(top_left[c*8+:8]>maximum) maximum=top_left[c*8+:8];
     out_data[c*8+:8]<=maximum;
    end
    out_valid<=1;
   end
   if(x==WIDTH-1) begin x<=0;y<=y+1'b1;end else x<=x+1'b1;
  end
 end
end
endmodule

