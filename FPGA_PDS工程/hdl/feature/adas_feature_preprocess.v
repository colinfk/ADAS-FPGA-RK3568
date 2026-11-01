// Streaming RGB888 preprocessing used by the FPGA feature branch.
// The video branch remains RGB565; this module is only a decision signal for
// the feature branch and must not replace the displayed video pixels.
module adas_feature_preprocess #(
    parameter integer WHITE_MIN = 8'd180,
    parameter integer WHITE_SPREAD = 8'd45
) (
    input  [7:0] r,
    input  [7:0] g,
    input  [7:0] b,
    output [7:0] gray,
    output       white_candidate
);
    wire [7:0] min_rgb = (r < g) ? ((r < b) ? r : b) : ((g < b) ? g : b);
    wire [7:0] max_rgb = (r > g) ? ((r > b) ? r : b) : ((g > b) ? g : b);
    wire [8:0] spread = {1'b0, max_rgb} - {1'b0, min_rgb};
    wire [9:0] gray_sum = {2'd0, r[7:2]} + {2'd0, g[7:1]} + {2'd0, b[7:2]};

    assign gray = gray_sum[7:0];
    assign white_candidate = (min_rgb >= WHITE_MIN) &&
                             (spread <= WHITE_SPREAD);
endmodule
