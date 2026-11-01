// Synchronous two-line buffer. Only complete interior windows are emitted.
// RAM is not reset: the first two rows/columns mask stale contents each frame.
module adas_window3x3 #(
    parameter integer WIDTH=1920,
    parameter integer BITS=8
) (
    input clk, rst_n, vs, de,
    input [BITS-1:0] pixel,
    output reg valid,
    output reg [11:0] x, y,
    output reg [BITS*9-1:0] window
);
reg [BITS-1:0] line1[0:WIDTH-1], line2[0:WIDTH-1];
reg [BITS-1:0] top, middle, bottom;
reg [BITS-1:0] t0,t1,m0,m1,b0,b1;
reg vs_d,de_d,read_valid;
reg [11:0] px,py,read_x,read_y;
wire start=vs && !vs_d;
wire [11:0] address=de_d ? px : 12'd0;
// Register the older-row write data/address. Each memory now has a simple
// synchronous read/write template rather than a memory-to-memory write path.
always @(posedge clk) if(de && !start && address<WIDTH) begin
    top<=line2[address]; middle<=line1[address]; bottom<=pixel;
    line1[address]<=pixel;
end
always @(posedge clk) if(read_valid && !start) line2[read_x]<=middle;
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        vs_d<=0;de_d<=0;read_valid<=0;valid<=0;px<=0;py<=0;
        read_x<=0;read_y<=0;x<=0;y<=0;window<=0;
        t0<=0;t1<=0;m0<=0;m1<=0;b0<=0;b1<=0;
    end else begin
        vs_d<=vs;de_d<=de;valid<=0;
        read_valid<=de && !start && address<WIDTH;
        if(start) begin
            px<=0;py<=0;read_valid<=0;
            t0<=0;t1<=0;m0<=0;m1<=0;b0<=0;b1<=0;
        end else begin
            if(de) begin
                read_x<=address;read_y<=py;px<=address+1'b1;
            end else if(de_d) begin px<=0;py<=py+1'b1;end
            if(read_valid) begin
                if(read_x==0) begin
                    t0<=0;t1<=top;m0<=0;m1<=middle;b0<=0;b1<=bottom;
                end else begin
                    t0<=t1;t1<=top;m0<=m1;m1<=middle;b0<=b1;b1<=bottom;
                    if(read_x>=2 && read_y>=2) begin
                        valid<=1;x<=read_x-1'b1;y<=read_y-1'b1;
                        window<={bottom,b1,b0,middle,m1,m0,top,t1,t0};
                    end
                end
            end
        end
    end
end
endmodule
