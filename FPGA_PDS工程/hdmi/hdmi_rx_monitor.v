// Observe the same sampled HDMI bundle as capture. Counters
// measure DE width/active lines; a clock alone never qualifies as valid video.
module hdmi_rx_monitor #(
    parameter integer CFG_HZ = 10000000,
    parameter integer WIDTH = 1920,
    parameter integer HEIGHT = 1080,
    parameter integer HEARTBEAT_BITS = 16,
    parameter integer INPUT_SAMPLED = 0
) (
    input cfg_clk, input pix_clk, input rst_n, input init_done,
    input vs, input hs, input de, input [7:0] r, g, b,
    output reg reconfigure,
    output [7:0] flags,
    output [127:0] geometry_diag,
    output reg [15:0] recovery_count
);
reg [HEARTBEAT_BITS-1:0] pix_count;
reg vs_d, de_d, vs_toggle, good_toggle;
reg vs_seen, de_seen, geometry_error;
reg [11:0] line_pixels, frame_lines, last_width, last_height;
reg line_bad;
reg [31:0] frame_count;
reg [7:0] pixel_signature;
reg hs_seen;
reg [63:0] frame_geometry;
wire vs_q,hs_q,de_q;
wire [7:0] r_q,g_q,b_q;
generate if (INPUT_SAMPLED) begin : g_shared_input
    assign {vs_q,hs_q,de_q,r_q,g_q,b_q}={vs,hs,de,r,g,b};
end else begin : g_local_input
    reg [26:0] sampled;
    always @(negedge pix_clk) sampled<={vs,hs,de,r,g,b};
    assign {vs_q,hs_q,de_q,r_q,g_q,b_q}=sampled;
end endgenerate
wire frame_start=vs_q && !vs_d;
wire line_end=de_d && !de_q;
wire [12:0] completed_lines={1'b0,frame_lines}+(line_end ? 13'd1 : 13'd0);
wire boundary_width_bad=line_end && line_pixels!=WIDTH;
// Configuration reset is asynchronous to the falling-edge pixel sampler.
// Assert immediately, then release only after two local sampling edges.
reg [1:0] pix_reset_pipe;
always @(posedge pix_clk or negedge rst_n)
    if (!rst_n) pix_reset_pipe <= 0;
    else pix_reset_pipe <= {pix_reset_pipe[0],1'b1};
wire pix_local_rst_n=pix_reset_pipe[1];
always @(posedge pix_clk or negedge pix_local_rst_n) begin
    if (!pix_local_rst_n) begin
        pix_count <= 0; vs_d <= 0; de_d <= 0; vs_toggle <= 0; good_toggle <= 0;
        vs_seen <= 0; de_seen <= 0; geometry_error <= 0; line_pixels <= 0;
        frame_lines <= 0; last_width <= 0; last_height <= 0; line_bad <= 0;
        frame_count <= 0; pixel_signature <= 0; hs_seen <= 0;
        frame_geometry<=0;
    end else begin
        pix_count <= pix_count + 1'b1; vs_d <= vs_q; de_d <= de_q;
        if (hs_q) hs_seen <= 1;
        if (de_q) begin
            de_seen <= 1;
            if (line_pixels != 12'hfff) line_pixels <= line_pixels + 1'b1;
            // Keep all physical RGB input bits used, including bits discarded
            // by RGB565. This signature is diagnostic, not a frame CRC.
            pixel_signature <= pixel_signature ^ r_q ^ g_q ^ b_q;
        end else if (de_d) begin
            last_width <= line_pixels; line_pixels <= 0;
            if (frame_lines != 12'hfff) frame_lines <= frame_lines + 1'b1;
            if (line_pixels != WIDTH) line_bad <= 1;
        end
        if (frame_start) begin
            vs_toggle <= ~vs_toggle; vs_seen <= 1; frame_count <= frame_count + 1'b1;
            last_height <= completed_lines[11:0]; frame_lines <= 0; line_bad <= 0;
            line_pixels<=0;
            frame_geometry<={frame_count+32'd1,completed_lines[11:0],
                line_end ? line_pixels : last_width,pixel_signature};
            pixel_signature<=0;
            // Ignore startup's incomplete first frame. Qualify later frames.
            if (vs_seen) begin
                if (completed_lines == HEIGHT && !line_bad && !boundary_width_bad &&
                    !de_q && (line_end || line_pixels==0))
                    good_toggle <= ~good_toggle;
                else geometry_error <= 1;
            end
        end
    end
end

reg [4:0] raw_d1, raw_d2;
reg [2:0] events_d1, events_d2, events_d3;
reg [31:0] clock_age, vs_age, good_age, recovery_age;
wire [31:0] geometry_d2, frames_d2;
cdc_snapshot #(.WIDTH(64)) u_geometry_snapshot (
    .src_clk(pix_clk),.src_rst_n(rst_n),.src_data(frame_geometry),
    .dst_clk(cfg_clk),.dst_rst_n(rst_n),.dst_data({frames_d2,geometry_d2}));
reg clock_seen;
reg hs_seen_d1, hs_seen_d2;
wire clock_event = events_d2[0] ^ events_d3[0];
wire vs_event = events_d2[1] ^ events_d3[1];
wire good_event = events_d2[2] ^ events_d3[2];
wire clock_live = clock_age < CFG_HZ/20;
wire video_live = init_done && clock_live && good_age < CFG_HZ/4;
always @(posedge cfg_clk or negedge rst_n) begin
    if (!rst_n) begin
        raw_d1 <= 0; raw_d2 <= 0; events_d1 <= 0; events_d2 <= 0; events_d3 <= 0;
        clock_age <= CFG_HZ; vs_age <= CFG_HZ; good_age <= CFG_HZ;
        recovery_age <= 0; reconfigure <= 0; recovery_count <= 0;
        clock_seen <= 0;
        hs_seen_d1 <= 0; hs_seen_d2 <= 0;
    end else begin
        raw_d1 <= {geometry_error, de_seen, vs_seen, 1'b1, init_done}; raw_d2 <= raw_d1;
        events_d1 <= {good_toggle, vs_toggle, pix_count[HEARTBEAT_BITS-1]};
        events_d2 <= events_d1; events_d3 <= events_d2;
        hs_seen_d1 <= hs_seen; hs_seen_d2 <= hs_seen_d1;
        reconfigure <= 0;
        if (clock_event) begin clock_age <= 0; clock_seen <= 1; end
        else if (clock_age < CFG_HZ) clock_age <= clock_age + 1'b1;
        if (vs_event) vs_age <= 0;
        else if (vs_age < CFG_HZ) vs_age <= vs_age + 1'b1;
        if (good_event && init_done) good_age <= 0;
        else if (!init_done) good_age <= CFG_HZ;
        else if (good_age < CFG_HZ) good_age <= good_age + 1'b1;
        // Missing VS/clock or wrong geometry for a full second requests the
        // existing seven-write output configuration. Do not invent chip bits.
        if (!init_done || good_event) recovery_age <= 0;
        else if (recovery_age == CFG_HZ-1) begin
            recovery_age <= 0; reconfigure <= 1;
            if (recovery_count != 16'hffff) recovery_count <= recovery_count + 1'b1;
        end else recovery_age <= recovery_age + 1'b1;
    end
end
// [0]init [1]clock_seen [2]vs_seen [3]de_seen [4]clock_live
// [5]video_live (correct geometry recently) [6]geometry_error [7]reserved.
assign flags = {1'b0, raw_d2[4], video_live, clock_live,
                raw_d2[3:2], clock_seen, init_done};
assign geometry_diag = {frames_d2, {15'd0, hs_seen_d2, recovery_count}, geometry_d2, 32'h52584432};
endmodule
