// First FPGA-side ADAS feature extractor.
//
// Input contract:
//   * one RGB888 pixel per clk when de=1;
//   * vs has a rising edge once per frame;
//   * mode_valid qualifies the whole frame.
//
// The output is a candidate result, not a semantic detector. A single
// continuous bright segment in the lower ROI is reported as a left/right
// lane-line candidate. Rows containing several separated bright segments are
// reported as a zebra-crossing candidate. The raw video path is untouched.
module adas_feature_top #(
    parameter integer WIDTH = 1920,
    parameter integer HEIGHT = 1080,
    parameter integer WHITE_MIN = 180,
    parameter integer WHITE_SPREAD = 45,
    parameter integer LANE_MIN_PIXELS = 4,
    parameter integer LANE_MAX_RUNS = 2,
    parameter integer LANE_MIN_ROWS = 8,
    parameter integer ZEBRA_MIN_PIXELS = 12,
    parameter integer ZEBRA_MIN_RUNS = 3,
    parameter integer ZEBRA_MIN_ROWS = 4,
    parameter integer EXTERNAL_FRAME_ID = 0
) (
    input             clk,
    input             rst_n,
    input             mode_valid,
    input             vs,
    input             de,
    input  [7:0]      r,
    input  [7:0]      g,
    input  [7:0]      b,
    input  [31:0]     source_frame_id,
    output reg        feature_valid,
    output reg [511:0] feature_packet,
    output reg [31:0] frame_id,
    output reg        left_valid,
    output reg        right_valid,
    output reg        zebra_valid,
    output reg [11:0] left_center_x,
    output reg [11:0] right_center_x,
    output reg [11:0] zebra_center_x,
    output reg [11:0] zebra_center_y
);
    localparam integer ROI_Y_START = (HEIGHT * 2) / 3;
    localparam integer HALF_WIDTH = WIDTH / 2;

    wire [7:0] gray_unused;
    wire white_pixel;
    adas_feature_preprocess #(
        .WHITE_MIN(WHITE_MIN), .WHITE_SPREAD(WHITE_SPREAD)
    ) u_preprocess (
        .r(r), .g(g), .b(b), .gray(gray_unused),
        .white_candidate(white_pixel)
    );

    reg vs_d, de_d;
    reg frame_seen;
    reg frame_qualified;
    reg [11:0] x_pos, y_pos;
    reg [11:0] line_x_min, line_x_max;
    reg [15:0] line_white_count;
    reg [11:0] line_left_min, line_left_max;
    reg [11:0] line_right_min, line_right_max;
    reg [15:0] line_left_count, line_right_count;
    reg [7:0] line_run_count;
    reg white_d;

    reg left_seen, right_seen, zebra_seen;
    reg [11:0] left_x_min, left_x_max, left_y_min, left_y_max;
    reg [11:0] right_x_min, right_x_max, right_y_min, right_y_max;
    reg [11:0] zebra_x_min, zebra_x_max, zebra_y_min, zebra_y_max;
    reg [15:0] left_rows, right_rows, zebra_rows;

    wire frame_start = vs && !vs_d;
    wire line_start = de && !de_d;
    wire line_end = !de && de_d;
    wire [11:0] line_left_center = (line_left_min + line_left_max) >> 1;
    wire [11:0] line_right_center = (line_right_min + line_right_max) >> 1;
    wire lane_left_line = (line_left_count >= LANE_MIN_PIXELS) &&
                          (line_run_count <= LANE_MAX_RUNS) &&
                          (y_pos >= ROI_Y_START);
    wire lane_right_line = (line_right_count >= LANE_MIN_PIXELS) &&
                           (line_run_count <= LANE_MAX_RUNS) &&
                           (y_pos >= ROI_Y_START);
    wire zebra_line = (line_white_count >= ZEBRA_MIN_PIXELS) &&
                      (line_run_count >= ZEBRA_MIN_RUNS) &&
                      (y_pos >= (HEIGHT / 3));

    wire [11:0] left_cx = (left_x_min + left_x_max) >> 1;
    wire [11:0] right_cx = (right_x_min + right_x_max) >> 1;
    wire [11:0] zebra_cx = (zebra_x_min + zebra_x_max) >> 1;
    wire [11:0] zebra_cy = (zebra_y_min + zebra_y_max) >> 1;
    wire left_confirmed = left_seen && (left_rows >= LANE_MIN_ROWS);
    wire right_confirmed = right_seen && (right_rows >= LANE_MIN_ROWS);
    wire zebra_confirmed = zebra_seen && (zebra_rows >= ZEBRA_MIN_ROWS);

    reg [127:0] left_record, right_record, zebra_record;
    always @(*) begin
        left_record = 128'd0;
        left_record[7:0] = 8'd1;
        left_record[15:8] = left_confirmed;
        left_record[31:16] = left_x_min;
        left_record[47:32] = left_y_min;
        left_record[63:48] = left_x_max;
        left_record[79:64] = left_y_max;
        left_record[95:80] = left_cx;
        left_record[111:96] = left_y_min + ((left_y_max - left_y_min) >> 1);
        left_record[119:112] = left_rows > 255 ? 8'd255 : left_rows[7:0];
        left_record[127:120] = 8'd180;

        right_record = 128'd0;
        right_record[7:0] = 8'd2;
        right_record[15:8] = right_confirmed;
        right_record[31:16] = right_x_min;
        right_record[47:32] = right_y_min;
        right_record[63:48] = right_x_max;
        right_record[79:64] = right_y_max;
        right_record[95:80] = right_cx;
        right_record[111:96] = right_y_min + ((right_y_max - right_y_min) >> 1);
        right_record[119:112] = right_rows > 255 ? 8'd255 : right_rows[7:0];
        right_record[127:120] = 8'd180;

        zebra_record = 128'd0;
        zebra_record[7:0] = 8'd3;
        zebra_record[15:8] = zebra_confirmed;
        zebra_record[31:16] = zebra_x_min;
        zebra_record[47:32] = zebra_y_min;
        zebra_record[63:48] = zebra_x_max;
        zebra_record[79:64] = zebra_y_max;
        zebra_record[95:80] = zebra_cx;
        zebra_record[111:96] = zebra_cy;
        zebra_record[119:112] = zebra_rows > 255 ? 8'd255 : zebra_rows[7:0];
        zebra_record[127:120] = 8'd160;
    end
    wire [7:0] packet_flags = {4'd0, frame_qualified,
                               zebra_confirmed, right_confirmed, left_confirmed};
    wire [511:0] packet_now;

    adas_feature_packet u_packet (
        .frame_id(EXTERNAL_FRAME_ID ? source_frame_id : frame_id), .image_width(WIDTH[15:0]),
        .image_height(HEIGHT[15:0]), .version(8'd1),
        .flags(packet_flags), .left_record(left_record),
        .right_record(right_record), .zebra_record(zebra_record),
        .packet(packet_now)
    );

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            vs_d <= 1'b0;
            de_d <= 1'b0;
            frame_seen <= 1'b0;
            frame_qualified <= 1'b0;
            frame_id <= 32'd0;
            feature_valid <= 1'b0;
            feature_packet <= 512'd0;
            x_pos <= 12'd0;
            y_pos <= 12'd0;
            line_x_min <= 12'd0;
            line_x_max <= 12'd0;
            line_white_count <= 16'd0;
            line_left_min <= 12'd0; line_left_max <= 12'd0;
            line_right_min <= 12'd0; line_right_max <= 12'd0;
            line_left_count <= 16'd0; line_right_count <= 16'd0;
            line_run_count <= 8'd0;
            white_d <= 1'b0;
            left_seen <= 1'b0;
            right_seen <= 1'b0;
            zebra_seen <= 1'b0;
            left_x_min <= 12'd0; left_x_max <= 12'd0;
            left_y_min <= 12'd0; left_y_max <= 12'd0;
            right_x_min <= 12'd0; right_x_max <= 12'd0;
            right_y_min <= 12'd0; right_y_max <= 12'd0;
            zebra_x_min <= 12'd0; zebra_x_max <= 12'd0;
            zebra_y_min <= 12'd0; zebra_y_max <= 12'd0;
            left_rows <= 16'd0; right_rows <= 16'd0; zebra_rows <= 16'd0;
        end else begin
            vs_d <= vs;
            de_d <= de;
            feature_valid <= 1'b0;

            // A frame boundary publishes the preceding frame and starts a
            // clean accumulation window for the next one.
            if (frame_start) begin
                if (frame_seen && frame_qualified) begin
                    feature_valid <= 1'b1;
                    feature_packet <= packet_now;
                end
                if (frame_seen)
                    frame_id <= frame_id + 1'b1;
                frame_seen <= 1'b1;
                frame_qualified <= mode_valid;
                x_pos <= 12'd0;
                y_pos <= 12'd0;
                line_x_min <= 12'd0;
                line_x_max <= 12'd0;
                line_white_count <= 16'd0;
                line_run_count <= 8'd0;
                white_d <= 1'b0;
                left_seen <= 1'b0; right_seen <= 1'b0; zebra_seen <= 1'b0;
                left_rows <= 16'd0; right_rows <= 16'd0; zebra_rows <= 16'd0;
            end else begin
                if (!mode_valid)
                    frame_qualified <= 1'b0;

                if (de && mode_valid) begin
                    if (line_start) begin
                        // The first active pixel belongs to the new line;
                        // seed all counters from this pixel explicitly.
                        x_pos <= 12'd1;
                        line_x_min <= white_pixel ? 12'd0 : WIDTH - 1;
                        line_x_max <= white_pixel ? 12'd0 : 12'd0;
                        line_white_count <= white_pixel ? 16'd1 : 16'd0;
                        line_run_count <= white_pixel ? 8'd1 : 8'd0;
                        white_d <= white_pixel;
                        line_left_min <= WIDTH - 1; line_left_max <= 12'd0;
                        line_right_min <= WIDTH - 1; line_right_max <= 12'd0;
                        line_left_count <= 16'd0; line_right_count <= 16'd0;
                        if (white_pixel) begin
                            if (12'd0 < HALF_WIDTH) begin
                                line_left_min <= 12'd0; line_left_max <= 12'd0;
                                line_left_count <= 16'd1;
                            end else begin
                                line_right_min <= 12'd0; line_right_max <= 12'd0;
                                line_right_count <= 16'd1;
                            end
                        end
                    end else begin
                        if (white_pixel) begin
                            if (line_white_count != 16'hffff)
                                line_white_count <= line_white_count + 1'b1;
                            if (!white_d && line_run_count != 8'hff)
                                line_run_count <= line_run_count + 1'b1;
                            if (line_white_count == 0 || x_pos < line_x_min)
                                line_x_min <= x_pos;
                            if (line_white_count == 0 || x_pos > line_x_max)
                                line_x_max <= x_pos;
                            if (x_pos < HALF_WIDTH) begin
                                if (line_left_count != 16'hffff)
                                    line_left_count <= line_left_count + 1'b1;
                                if (line_left_count == 0 || x_pos < line_left_min)
                                    line_left_min <= x_pos;
                                if (line_left_count == 0 || x_pos > line_left_max)
                                    line_left_max <= x_pos;
                            end else begin
                                if (line_right_count != 16'hffff)
                                    line_right_count <= line_right_count + 1'b1;
                                if (line_right_count == 0 || x_pos < line_right_min)
                                    line_right_min <= x_pos;
                                if (line_right_count == 0 || x_pos > line_right_max)
                                    line_right_max <= x_pos;
                            end
                            white_d <= 1'b1;
                        end else begin
                            white_d <= 1'b0;
                        end
                        if (x_pos != 12'hfff)
                            x_pos <= x_pos + 1'b1;
                    end
                end

                // The last active pixel was counted on the preceding clock,
                // so the line accumulators are complete at DE falling edge.
                if (line_end && mode_valid) begin
                    if (lane_left_line) begin
                        if (!left_seen) begin
                                left_x_min <= line_left_min; left_x_max <= line_left_max;
                                left_y_min <= y_pos; left_y_max <= y_pos;
                        end else begin
                                if (line_left_min < left_x_min) left_x_min <= line_left_min;
                                if (line_left_max > left_x_max) left_x_max <= line_left_max;
                                left_y_max <= y_pos;
                        end
                        left_seen <= 1'b1;
                        if (left_rows != 16'hffff) left_rows <= left_rows + 1'b1;
                    end
                    if (lane_right_line) begin
                        if (!right_seen) begin
                            right_x_min <= line_right_min; right_x_max <= line_right_max;
                            right_y_min <= y_pos; right_y_max <= y_pos;
                        end else begin
                            if (line_right_min < right_x_min) right_x_min <= line_right_min;
                            if (line_right_max > right_x_max) right_x_max <= line_right_max;
                            right_y_max <= y_pos;
                        end
                        right_seen <= 1'b1;
                        if (right_rows != 16'hffff) right_rows <= right_rows + 1'b1;
                    end
                    if (zebra_line) begin
                        if (!zebra_seen) begin
                            zebra_x_min <= line_x_min; zebra_x_max <= line_x_max;
                            zebra_y_min <= y_pos; zebra_y_max <= y_pos;
                        end else begin
                            if (line_x_min < zebra_x_min) zebra_x_min <= line_x_min;
                            if (line_x_max > zebra_x_max) zebra_x_max <= line_x_max;
                            zebra_y_max <= y_pos;
                        end
                        zebra_seen <= 1'b1;
                        if (zebra_rows != 16'hffff) zebra_rows <= zebra_rows + 1'b1;
                    end
                    y_pos <= (y_pos == 12'hfff) ? y_pos : y_pos + 1'b1;
                end
            end
        end
    end

    always @(*) begin
        left_valid = left_confirmed;
        right_valid = right_confirmed;
        zebra_valid = zebra_confirmed;
        left_center_x = left_cx;
        right_center_x = right_cx;
        zebra_center_x = zebra_cx;
        zebra_center_y = zebra_cy;
    end
endmodule
