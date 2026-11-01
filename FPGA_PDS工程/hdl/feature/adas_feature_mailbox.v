// Small register/BRAM-friendly holding register for the latest feature packet.
// It is intentionally independent from the existing video DDR ABI. The next
// integration step can connect packet_latched to a DDR slot or PCIe BAR/DMA.
module adas_feature_mailbox (
    input             clk,
    input             rst_n,
    input             packet_valid,
    input  [511:0]    packet_in,
    output reg        packet_latched_valid,
    output reg [31:0] packet_sequence,
    output reg [511:0] packet_latched
);
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            packet_latched_valid <= 1'b0;
            packet_sequence <= 32'd0;
            packet_latched <= 512'd0;
        end else if (packet_valid) begin
            packet_latched_valid <= 1'b1;
            packet_sequence <= packet_sequence + 1'b1;
            packet_latched <= packet_in;
        end
    end
endmodule
