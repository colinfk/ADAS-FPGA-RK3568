// Companion metadata for the unchanged complete RGB565 image. Counts are
// geometric/color candidates, never traffic-light or obstacle classifications.
module adas_road_stats #(
    parameter integer WIDTH=1920, HEIGHT=1080,
    parameter integer SOBEL_THRESHOLD=96
) (
    input clk,rst_n,mode_valid,vs,de,
    input [7:0] r,g,b,
    input [31:0] source_frame_id,
    output reg stats_valid,
    output reg [1023:0] stats_packet
);
wire [7:0] gray;
wire white;
adas_feature_preprocess pre(.r(r),.g(g),.b(b),.gray(gray),.white_candidate(white));
wire win_valid;
wire [11:0] wx,wy;
wire [71:0] w;
adas_window3x3 #(.WIDTH(WIDTH)) gray_window(
    .clk(clk),.rst_n(rst_n),.vs(vs),.de(de && mode_valid),.pixel(gray),
    .valid(win_valid),.x(wx),.y(wy),.window(w));
wire [10:0] gx_left={3'd0,w[7:0]}+({3'd0,w[31:24]}<<1)+{3'd0,w[55:48]};
wire [10:0] gx_right={3'd0,w[23:16]}+({3'd0,w[47:40]}<<1)+{3'd0,w[71:64]};
wire [10:0] gy_top={3'd0,w[7:0]}+({3'd0,w[15:8]}<<1)+{3'd0,w[23:16]};
wire [10:0] gy_bottom={3'd0,w[55:48]}+({3'd0,w[63:56]}<<1)+{3'd0,w[71:64]};
wire [10:0] ax=(gx_left>gx_right) ? gx_left-gx_right : gx_right-gx_left;
wire [10:0] ay=(gy_top>gy_bottom) ? gy_top-gy_bottom : gy_bottom-gy_top;
wire edge_pixel=({1'b0,ax}+{1'b0,ay})>=SOBEL_THRESHOLD;
wire [11:0] gaussian_sum={4'd0,w[7:0]}+({4'd0,w[15:8]}<<1)+{4'd0,w[23:16]}+
    ({4'd0,w[31:24]}<<1)+({4'd0,w[39:32]}<<2)+({4'd0,w[47:40]}<<1)+
    {4'd0,w[55:48]}+({4'd0,w[63:56]}<<1)+{4'd0,w[71:64]};
wire morph_valid;
wire [11:0] mx,my;
wire [8:0] mw;
adas_window3x3 #(.WIDTH(WIDTH-2),.BITS(1)) binary_window(
    .clk(clk),.rst_n(rst_n),.vs(vs),.de(win_valid),.pixel(edge_pixel),
    .valid(morph_valid),.x(mx),.y(my),.window(mw));
wire [11:0] morph_x=mx+1'b1,morph_y=my+1'b1;
reg vs_d,de_d,seen,qualified;
reg [11:0] x,y;
reg [31:0] count[0:23];
reg [1023:0] packet_now;
integer i;
integer j;
localparam integer HALF_WIDTH=WIDTH/2, LANE_Y=(HEIGHT*2)/3, ZEBRA_Y=HEIGHT/3;
wire start=vs && !vs_d;
wire [11:0] ix=de_d ? x : 12'd0;
wire red=(r>=160) && ({1'b0,r}>={1'b0,g}+9'd48) && ({1'b0,r}>={1'b0,b}+9'd48);
wire green=(g>=160) && ({1'b0,g}>={1'b0,r}+9'd48) && ({1'b0,g}>={1'b0,b}+9'd48);
wire blue=(b>=160) && ({1'b0,b}>={1'b0,r}+9'd48) && ({1'b0,b}>={1'b0,g}+9'd48);
always @(*) begin
    packet_now=0;
    packet_now[31:0]=32'h31584652; // RFX1
    packet_now[63:32]=source_frame_id;
    packet_now[95:64]=32'd1;
    packet_now[127:96]=(qualified && count[0]==WIDTH*HEIGHT && y==HEIGHT) ? 32'h7f : 32'h7e;
    for(j=0;j<20;j=j+1) packet_now[(j+4)*32+:32]=count[j];
    packet_now[799:768]={8'd45,8'd180,8'd64,SOBEL_THRESHOLD[7:0]};
    packet_now[831:800]={HALF_WIDTH[15:0],LANE_Y[15:0]};
    packet_now[863:832]={HEIGHT[15:0],ZEBRA_Y[15:0]};
    packet_now[895:864]={HEIGHT[15:0],WIDTH[15:0]};
    packet_now[927:896]=count[20];
    packet_now[959:928]=count[21];
end
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        vs_d<=0;de_d<=0;seen<=0;qualified<=0;x<=0;y<=0;
        stats_valid<=0;stats_packet<=0;
        for(i=0;i<24;i=i+1) count[i]<=0;
    end else begin
        vs_d<=vs;de_d<=de;stats_valid<=0;
        if(start) begin
            if(seen) begin stats_valid<=1;stats_packet<=packet_now;end
            seen<=1;qualified<=mode_valid;x<=0;y<=0;
            for(i=0;i<24;i=i+1) count[i]<=0;
        end else begin
            if(!mode_valid) qualified<=0;
            if(de && mode_valid) begin
                x<=ix+1'b1;
                count[0]<=count[0]+1'b1;
                count[1]<=count[1]+gray;
                if(gray<64) count[2]<=count[2]+1'b1;
                if(gray>=180) count[3]<=count[3]+1'b1;
                if(white && y>=(HEIGHT*2)/3) begin
                    if(ix<WIDTH/2) count[4]<=count[4]+1'b1;
                    else count[5]<=count[5]+1'b1;
                end
                if(white && y>=HEIGHT/3) count[6]<=count[6]+1'b1;
                if(red) count[12]<=count[12]+1'b1;
                if(green) count[13]<=count[13]+1'b1;
                if(blue) count[14]<=count[14]+1'b1;
            end
            if(!de && de_d) begin
                if(x!=WIDTH) qualified<=0;
                x<=0;y<=y+1'b1;
            end
            if(win_valid) begin
                count[7]<=count[7]+gaussian_sum[11:4];
                count[20]<=count[20]+1'b1;
                if(edge_pixel) begin
                    count[8]<=count[8]+1'b1;
                    if(wy>=(HEIGHT*2)/3) begin
                        if(wx<WIDTH/2) count[15]<=count[15]+1'b1;
                        else count[16]<=count[16]+1'b1;
                    end
                    if(wy>=HEIGHT/3) count[17]<=count[17]+1'b1;
                end
            end
            if(morph_valid) begin
                count[21]<=count[21]+1'b1;
                if(&mw) count[10]<=count[10]+1'b1;
                if(|mw) begin
                    count[9]<=count[9]+1'b1;
                    if(morph_y>=(HEIGHT*2)/3) begin
                        if(morph_x<WIDTH/2) count[18]<=count[18]+1'b1;
                        else count[19]<=count[19]+1'b1;
                    end
                    if(morph_y>=HEIGHT/3) count[11]<=count[11]+1'b1;
                end
            end
        end
    end
end
endmodule
