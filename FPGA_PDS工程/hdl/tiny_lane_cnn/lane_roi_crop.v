// ROI v4: 512-row perspective corridor; four horizontal pixels per output.
module lane_roi_crop #(
 parameter LEFT_X=480,RIGHT_X=1312,TOP_Y=512,
 parameter SX_LOG2=2,SY_LOG2=3
)(
 input clk,rst_n,frame_start,pixel_valid,input [11:0] x,y,input [15:0] rgb565,
 output reg left_valid,right_valid,output reg [10:0] left_addr,right_addr,
 output reg [7:0] left_gray,right_gray
);
localparam ROI_H=64*(1<<SY_LOG2),ROI_W=32*(1<<SX_LOG2);
reg [9:0] left_sum,right_sum;
wire [7:0] r={rgb565[15:11],rgb565[15:13]};
wire [7:0] g={rgb565[10:5],rgb565[10:9]};
wire [7:0] b={rgb565[4:0],rgb565[4:2]};
wire [9:0] gray_sum={2'd0,r}+{1'd0,g,1'b0}+{2'd0,b};
wire [7:0] gray=gray_sum[9:2];
wire [11:0] dy=y-TOP_Y;
wire [11:0] remain=(ROI_H-1)-dy;
wire [11:0] shift=(remain>>1)+(remain>>3);
wire [11:0] left_x0=LEFT_X+shift,right_x0=RIGHT_X-shift;
wire [11:0] ldx=x-left_x0,rdx=x-right_x0;
reg [9:0] total;
always @(posedge clk) begin
 if(!rst_n || frame_start) begin
  left_valid<=0;right_valid<=0;left_addr<=0;right_addr<=0;
  left_gray<=0;right_gray<=0;left_sum<=0;right_sum<=0;
 end else begin
  left_valid<=0;right_valid<=0;
  if(pixel_valid && y>=TOP_Y && y<TOP_Y+ROI_H && (dy&7)==4) begin
   if(x>=left_x0 && x<left_x0+ROI_W) begin
    if(ldx[1:0]==0)left_sum<=gray;
    else if(ldx[1:0]==3) begin
     total=left_sum+gray+10'd2;
     left_valid<=1;left_gray<=total[9:2];left_addr<={dy[8:3],ldx[6:2]};
    end else left_sum<=left_sum+gray;
   end
   if(x>=right_x0 && x<right_x0+ROI_W) begin
    if(rdx[1:0]==0)right_sum<=gray;
    else if(rdx[1:0]==3) begin
     total=right_sum+gray+10'd2;
     right_valid<=1;right_gray<=total[9:2];right_addr<={dy[8:3],rdx[6:2]};
    end else right_sum<=right_sum+gray;
   end
  end
 end
end
endmodule
