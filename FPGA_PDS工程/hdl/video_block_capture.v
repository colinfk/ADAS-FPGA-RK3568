module video_block_capture #(
    parameter integer WIDTH       = 1920,
    parameter integer HEIGHT      = 1080,
    parameter integer CHUNK_WORDS = 256,
    parameter integer SLOT_COUNT  = 15,
    parameter integer INPUT_SAMPLED = 0
) (
    input               pixclk,
    input               rst_n,
    input               rx_ready,
    input               vs,
    input               de,
    input       [7:0]   r,
    input       [7:0]   g,
    input       [7:0]   b,
    output reg          wr_en,
    output reg  [11:0]  wr_addr,
    output reg  [127:0] wr_data,
    output reg          capture_done,
    output reg          frame_epoch,
    // Stable frame stream boundary for the DDR writer. The legacy wr_* port
    // remains unchanged for the direct-stream benchmark path.
    output reg          frame_stream_valid,
    output reg  [127:0]  frame_stream_data,
    output reg          frame_stream_epoch,
    output reg          frame_stream_sof,
output reg          frame_stream_eof,
output reg  [19:0]   frame_stream_word_index,
    // DDR production stream: eight RGB565 pixels per 128-bit word. The
    // legacy XRGB8888 stream above remains unchanged for direct-path debug.
    output reg          ddr_stream_valid,
    output reg  [127:0] ddr_stream_data,
    output reg          ddr_stream_epoch,
    output reg          ddr_stream_sof,
    output reg          ddr_stream_eof,
    output reg  [19:0]  ddr_stream_word_index,
    output reg          ddr_stream_abort,
    output reg [31:0]   ddr_stream_source_frame_id,
    output [31:0]       source_frame_id_out,
    output [127:0]      capture_diag,
    output [127:0]      geometry_fault_diag
);

localparam integer FRAME_WORDS = (WIDTH * HEIGHT) / 4;
localparam integer DDR_FRAME_WORDS = (WIDTH * HEIGHT) / 8;

reg         vs_d;
wire        vs_sampled;
wire        de_sampled;
reg         rx_ready_sampled;
wire [7:0] r_sampled, g_sampled, b_sampled;
reg de_d;
reg [11:0] line_pixels, active_lines;
reg ddr_eof_pending;
reg [127:0] ddr_eof_data;
reg geometry_active, geometry_bad;
reg [31:0] width_error_count, height_error_count, rejected_count;
assign capture_diag = {rejected_count, height_error_count, width_error_count, 32'h43415031};
reg         frame_active;
reg [1:0]   pixel_count;
reg [31:0]  pixel0;
reg [31:0]  pixel1;
reg [31:0]  pixel2;
reg [11:0]  x;
reg [11:0]  y;
reg [7:0]   chunk_word_count;
reg [3:0]   slot_index;
reg [19:0]  frame_word_count;
reg [2:0]   ddr_pixel_count;
reg [111:0] ddr_pixel_pack;
reg [19:0]  ddr_word_count;

wire [31:0] current_pixel = {8'h00, r_sampled, g_sampled, b_sampled};
wire [15:0] current_rgb565 = {r_sampled[7:3], g_sampled[7:2], b_sampled[7:3]};
wire        frame_start   = vs_sampled && !vs_d;
wire line_end = de_d && !de_sampled && geometry_active;
// Include a DE falling edge on the same sample as VS in the OLD frame.
// All decisions use this next value; the new frame still starts empty.
wire [12:0] completed_lines = {1'b0,active_lines} + (line_end ? 13'd1 : 13'd0);
wire boundary_width_bad = line_end && (line_pixels!=WIDTH || active_lines>=HEIGHT);
wire old_frame_valid = !geometry_bad && !boundary_width_bad && !de_sampled &&
                       completed_lines==HEIGHT && (line_end || line_pixels==0) && ddr_eof_pending;
reg [31:0] source_frame_id;
assign source_frame_id_out=source_frame_id;
reg fault_seen;
reg [127:0] first_geometry_fault;
assign geometry_fault_diag=first_geometry_fault;
task record_fault;
    input [7:0] reason;
    input [11:0] actual_width;
    input [12:0] actual_height;
    begin
        if (!fault_seen) begin
            fault_seen<=1;
            first_geometry_fault<={source_frame_id,
                9'd0,rx_ready_sampled,vs_sampled,de_sampled,active_lines,reason,
                3'd0,actual_height,4'd0,actual_width,32'h47454f31};
        end
    end
endtask

// Capture all parallel inputs together; never combine half-cycle-old DE
// with unsampled RGB. Physical phase/readback must be checked on the board.
// RGB has no reset: it is only consumed with the simultaneously sampled,
// reset-qualified controls. This avoids a positive-edge reset-release path
// to the negative-edge RGB registers and preserves normal lane alignment.
generate if (INPUT_SAMPLED) begin : g_shared_input
    assign {vs_sampled,de_sampled,r_sampled,g_sampled,b_sampled}={vs,de,r,g,b};
end else begin : g_local_input
    reg [23:0] rgb_q;
    reg vs_q, de_q;
    always @(negedge pixclk) rgb_q<={r,g,b};
    always @(negedge pixclk or negedge rst_n)
        if (!rst_n) begin vs_q<=0; de_q<=0; end
        else begin vs_q<=vs; de_q<=de; end
    assign {vs_sampled,de_sampled,r_sampled,g_sampled,b_sampled}={vs_q,de_q,rgb_q};
end endgenerate
always @(negedge pixclk or negedge rst_n) begin
    if (!rst_n) begin
        rx_ready_sampled <= 1'b0;
    end else begin
        rx_ready_sampled <= rx_ready;
    end
end

always @(posedge pixclk or negedge rst_n) begin
    if (!rst_n) begin
        vs_d             <= 1'b0;
        de_d <= 0; line_pixels <= 0; active_lines <= 0;
        ddr_eof_pending <= 0; ddr_eof_data <= 0;
        geometry_active<=0; geometry_bad<=0;
        width_error_count<=0; height_error_count<=0; rejected_count<=0;
        source_frame_id<=0; fault_seen<=0; first_geometry_fault<=128'h47454f31;
        ddr_stream_source_frame_id<=0;
        ddr_stream_abort<=0;
        frame_active     <= 1'b0;
        pixel_count      <= 2'd0;
        pixel0           <= 32'd0;
        pixel1           <= 32'd0;
        pixel2           <= 32'd0;
        x                <= 12'd0;
        y                <= 12'd0;
        chunk_word_count <= 8'd0;
        slot_index       <= 4'd0;
        frame_word_count <= 20'd0;
        wr_en            <= 1'b0;
        wr_addr          <= 12'd0;
        wr_data          <= 128'd0;
        capture_done     <= 1'b0;
        frame_epoch      <= 1'b0;
        frame_stream_valid <= 1'b0;
        frame_stream_data <= 128'd0;
        frame_stream_epoch <= 1'b0;
        frame_stream_sof <= 1'b0;
        frame_stream_eof <= 1'b0;
        frame_stream_word_index <= 20'd0;
        ddr_stream_valid <= 1'b0;
        ddr_stream_data <= 128'd0;
        ddr_stream_epoch <= 1'b0;
        ddr_stream_sof <= 1'b0;
        ddr_stream_eof <= 1'b0;
        ddr_stream_abort <= 0;
        ddr_stream_word_index <= 20'd0;
        ddr_pixel_count <= 3'd0;
        ddr_pixel_pack <= 112'd0;
        ddr_word_count <= 20'd0;
    end else begin
        vs_d         <= vs_sampled;
        ddr_stream_source_frame_id<=source_frame_id;
        de_d <= de_sampled;
        wr_en        <= 1'b0;
        capture_done <= 1'b0;
        frame_stream_valid <= 1'b0;
        frame_stream_sof <= 1'b0;
        frame_stream_eof <= 1'b0;
        ddr_stream_valid <= 1'b0;
        ddr_stream_sof <= 1'b0;
        ddr_stream_eof <= 1'b0;

        if (!rx_ready_sampled) begin
            ddr_stream_abort<=0;
            if (geometry_active) begin
                ddr_stream_valid<=1; ddr_stream_abort<=1;
                rejected_count<=rejected_count+1'b1;
                record_fault(8'd4,line_pixels,{1'b0,active_lines});
            end
            geometry_active<=0; geometry_bad<=0;
            frame_active     <= 1'b0;
            line_pixels <= 0; active_lines <= 0; ddr_eof_pending <= 0;
            pixel_count      <= 2'd0;
            x                <= 12'd0;
            y                <= 12'd0;
            chunk_word_count <= 8'd0;
            slot_index       <= 4'd0;
            frame_word_count <= 20'd0;
            ddr_pixel_count  <= 3'd0;
            ddr_word_count   <= 20'd0;
        end else begin
            ddr_stream_abort<=0;
            if (geometry_active && de_sampled && !frame_start) begin
                if (line_pixels != 12'hfff) line_pixels <= line_pixels + 1'b1;
                if (vs_sampled) begin
                    geometry_bad<=1;
                    record_fault(8'd3,line_pixels,{1'b0,active_lines});
                end
            end
            if (line_end) begin
                line_pixels <= 0;
                if (active_lines != 12'hfff) active_lines <= active_lines + 1'b1;
                if (line_pixels != WIDTH || active_lines >= HEIGHT) begin
                    // Continue tracking geometry until VS, even after payload
                    // completion. Abort is a distinct token, not writer corruption.
                    geometry_bad<=1;
                    if (line_pixels != WIDTH) width_error_count<=width_error_count+1'b1;
                    record_fault(line_pixels!=WIDTH ? 8'd1 : 8'd2,line_pixels,completed_lines);
                    frame_active <= 0; ddr_eof_pending <= 0;
                    pixel_count <= 0; ddr_pixel_count <= 0;
                end
            end
            if (frame_start) begin
                source_frame_id<=source_frame_id+1'b1;
                if (geometry_active) begin
                    ddr_stream_valid<=1;
                    ddr_stream_epoch<=frame_epoch;
                    ddr_stream_word_index<=DDR_FRAME_WORDS-1;
                    if (old_frame_valid) begin
                        ddr_stream_data<=ddr_eof_data;
                        ddr_stream_sof<=(DDR_FRAME_WORDS==1);
                        ddr_stream_eof<=1;
                    end else begin
                        ddr_stream_abort<=1;
                        rejected_count<=rejected_count+1'b1;
                        if (completed_lines!=HEIGHT || de_sampled)
                            height_error_count<=height_error_count+1'b1;
                        // A width fault at this boundary has priority over height.
                        if (!boundary_width_bad)
                            record_fault(de_sampled ? 8'd3 : 8'd2,line_pixels,completed_lines);
                    end
                end
                frame_active     <= !de_sampled;
                geometry_active<=1; geometry_bad<=de_sampled;
                line_pixels <= 0; active_lines <= 0; ddr_eof_pending <= 0;
                frame_epoch      <= ~frame_epoch;
                pixel_count      <= 2'd0;
                x                <= 12'd0;
                y                <= 12'd0;
                chunk_word_count <= 8'd0;
                slot_index       <= 4'd0;
                frame_word_count <= 20'd0;
                ddr_pixel_count  <= 3'd0;
                ddr_word_count   <= 20'd0;
                ddr_pixel_pack   <= 112'd0;
            end

            if (frame_active && de_sampled && !vs_sampled && !frame_start && line_pixels < WIDTH) begin
                case (pixel_count)
                    2'd0: begin pixel0 <= current_pixel; pixel_count <= 2'd1; end
                    2'd1: begin pixel1 <= current_pixel; pixel_count <= 2'd2; end
                    2'd2: begin pixel2 <= current_pixel; pixel_count <= 2'd3; end
                    default: begin
                        // Lane 0 is the first pixel in little-endian memory order.
                        wr_en       <= 1'b1;
                        wr_addr     <= {slot_index, chunk_word_count};
                        wr_data     <= {current_pixel, pixel2, pixel1, pixel0};
                        frame_stream_valid <= 1'b1;
                        frame_stream_data <= {current_pixel, pixel2, pixel1, pixel0};
                        frame_stream_epoch <= frame_epoch;
                        frame_stream_sof <= (frame_word_count == 20'd0);
                        frame_stream_eof <= (frame_word_count == FRAME_WORDS - 1);
                        frame_stream_word_index <= frame_word_count;
                        pixel_count <= 2'd0;

                        if (x == WIDTH - 4) begin
                            x <= 12'd0;
                            if (y == HEIGHT - 1)
                                y <= 12'd0;
                            else
                                y <= y + 12'd1;
                        end else begin
                            x <= x + 12'd4;
                        end

                        if (frame_word_count == FRAME_WORDS - 1) begin
                            frame_active     <= 1'b0;
                            capture_done     <= 1'b1;
                            frame_word_count <= 20'd0;
                            chunk_word_count <= 8'd0;
                            slot_index       <= 4'd0;
                        end else begin
                            frame_word_count <= frame_word_count + 20'd1;
                            if (chunk_word_count == CHUNK_WORDS - 1) begin
                                chunk_word_count <= 8'd0;
                                if (slot_index == SLOT_COUNT - 1)
                                    slot_index <= 4'd0;
                                else
                                    slot_index <= slot_index + 4'd1;
                            end else begin
                                chunk_word_count <= chunk_word_count + 8'd1;
                            end
                        end
                    end
                endcase
            end

            // Independent RGB565 packer for the DDR production route.
            // Pixel 0 occupies bits [15:0], matching the legacy little-endian
            // lane convention. Only active-video pixels are accepted.
            if (frame_active && de_sampled && !vs_sampled && !frame_start && line_pixels < WIDTH) begin
                case (ddr_pixel_count)
                    3'd0: begin ddr_pixel_pack[15:0]   <= current_rgb565; ddr_pixel_count <= 3'd1; end
                    3'd1: begin ddr_pixel_pack[31:16]  <= current_rgb565; ddr_pixel_count <= 3'd2; end
                    3'd2: begin ddr_pixel_pack[47:32]  <= current_rgb565; ddr_pixel_count <= 3'd3; end
                    3'd3: begin ddr_pixel_pack[63:48]  <= current_rgb565; ddr_pixel_count <= 3'd4; end
                    3'd4: begin ddr_pixel_pack[79:64]  <= current_rgb565; ddr_pixel_count <= 3'd5; end
                    3'd5: begin ddr_pixel_pack[95:80]  <= current_rgb565; ddr_pixel_count <= 3'd6; end
                    3'd6: begin ddr_pixel_pack[111:96] <= current_rgb565; ddr_pixel_count <= 3'd7; end
                    default: begin
                        ddr_stream_valid <= 1'b1;
                        ddr_stream_data <= {current_rgb565, ddr_pixel_pack};
                        ddr_stream_epoch <= frame_epoch;
                        ddr_stream_sof <= (ddr_word_count == 20'd0);
                        ddr_stream_eof <= (ddr_word_count == DDR_FRAME_WORDS - 1);
                        ddr_stream_word_index <= ddr_word_count;
                        ddr_pixel_count <= 3'd0;
                        if (ddr_word_count == DDR_FRAME_WORDS - 1) begin
                            // Hold until next VS validates the entire frame,
                            // including any extra active lines after HEIGHT.
                            ddr_stream_valid <= 0;
                            ddr_stream_eof <= 0;
                            ddr_eof_pending <= 1;
                            ddr_eof_data <= {current_rgb565, ddr_pixel_pack};
                            ddr_word_count <= 20'd0;
                        end else begin
                            ddr_word_count <= ddr_word_count + 20'd1;
                        end
                    end
                endcase
            end
        end
    end
end

endmodule
