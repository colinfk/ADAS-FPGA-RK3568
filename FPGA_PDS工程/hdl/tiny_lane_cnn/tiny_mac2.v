// Verilog-2001, two output-channel lanes sharing one activation.
// clear seeds both accumulators from bias; it has priority over step.
// step[1:0] masks the lanes independently (FC's final group uses one lane).
module tiny_mac2 (
    input wire clk,
    input wire rst_n,
    input wire clear,
    input wire [1:0] step,
    input wire signed [7:0] activation,
    input wire signed [7:0] weight0,
    input wire signed [7:0] weight1,
    input wire signed [31:0] bias0,
    input wire signed [31:0] bias1,
    output reg signed [31:0] acc0,
    output reg signed [31:0] acc1
);
wire signed [15:0] product0 = activation * weight0;
wire signed [15:0] product1 = activation * weight1;
wire signed [31:0] extended0 = {{16{product0[15]}}, product0};
wire signed [31:0] extended1 = {{16{product1[15]}}, product1};
always @(posedge clk) begin
    if (!rst_n) begin
        acc0 <= 0;
        acc1 <= 0;
    end else if (clear) begin
        acc0 <= bias0;
        acc1 <= bias1;
    end else begin
        if (step[0]) acc0 <= acc0 + extended0;
        if (step[1]) acc1 <= acc1 + extended1;
    end
end
endmodule

