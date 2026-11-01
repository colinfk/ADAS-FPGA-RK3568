// Board self-test source, not an external HDMI acceptance test.
// sys_clk=25 MHz gives about 10.1 frames/s for this 2200x1125 timing.
module video_pipeline_test_source (
    input clk,rst_n,
    output wire vs,hs,de,
    output wire [7:0] r,g,b
);
reg [11:0] x,y;
reg [7:0] frame;
always @(posedge clk or negedge rst_n) begin
    if(!rst_n)begin x<=0;y<=0;frame<=0;end
    else if(x==2199)begin
        x<=0;
        if(y==1124)begin y<=0;frame<=frame+1'b1;end
        else y<=y+1'b1;
    end else x<=x+1'b1;
end
assign vs=y>=1084 && y<1089;
assign hs=x>=2008 && x<2052;
assign de=x<1920 && y<1080;
wire lane=y>=720 && ((x>=450 && x<466)||(x>=1450 && x<1466));
wire zebra=y>=400 && y<480 && ((x>=820 && x<836)||(x>=900 && x<916)||(x>=980 && x<996));
wire car=y>=600 && y<660 && x>=900 && x<1000;
wire [7:0] dark=8'd16+{5'd0,frame[2:0]};
assign r=!de ? 0 : ((lane||zebra) ? 8'd200 : (car ? 8'd80 : dark));
assign g=!de ? 0 : ((lane||zebra) ? 8'd200 : (car ? 8'd20 : dark));
assign b=!de ? 0 : ((lane||zebra) ? 8'd200 : dark);
endmodule
