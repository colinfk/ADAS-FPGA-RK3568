`timescale 1ns / 1ps
// Single MS7200 receiver; no MS7210 controller or shared-bus arbitration.
module ms72xx_ctl(
    input clk, input rst_n, input reconfigure, input video_valid,
    output init_over_rx, output init_over,
    output iic_scl, inout iic_sda,
    output error_seen, output [15:0] error_count, last_error_addr,
    output [31:0] freq_rec, output [7:0] phase_readback,
    output [4:0] debug_state, output [127:0] config_diag,
    input [31:0] control_request, output [127:0] control_diag, output sample_rising
);
reg [1:0] reset_sync;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) reset_sync <= 0;
    else reset_sync <= {reset_sync[0], 1'b1};
end
wire rstn = reset_sync[1];
wire iic_trig, w_r, busy, byte_over, txn_error, iic_abort;
wire [7:0] device_id, data_in, data_out;
wire [15:0] addr;
ms7200_ctl U1_ms7200_ctl (
    .clk(clk), .rstn(rstn), .init_over(init_over_rx), .device_id(device_id),
    .iic_trig(iic_trig), .w_r(w_r), .addr(addr), .data_in(data_in),
    .busy(busy), .data_out(data_out), .byte_over(byte_over),
    .txn_error(txn_error), .reconfigure(reconfigure), .iic_abort(iic_abort),
    .video_valid(video_valid), .config_diag(config_diag),
    .control_request(control_request),.control_diag(control_diag),.sample_rising(sample_rising),
    .error_seen(error_seen), .error_count(error_count), .last_error_addr(last_error_addr),
    .freq_rec(freq_rec), .phase_readback(phase_readback), .debug_state(debug_state)
);
assign init_over = init_over_rx;
wire sda_out, sda_out_en;
reg [1:0] sda_sync;
always @(posedge clk or negedge rstn) begin
    if (!rstn) sda_sync <= 2'b11;
    else sda_sync <= {sda_sync[0], iic_sda};
end
// Match the supplied receiver example's actual 400 kHz / 1 us cadence.
// That example divides its nominal T_WR value by 1,000,000, not 1,000.
iic_dri #(.CLK_FRE(10000000), .IIC_FREQ(400000), .T_WR(0), .POST_TRANSACTION_US(1),
    .ADDR_BYTE(2), .LEN_WIDTH(3), .DATA_BYTE(1)) iic_dri (
    .clk(clk), .rstn(rstn && !iic_abort), .device_id(device_id),
    .pluse(iic_trig), .w_r(w_r), .byte_len(4'd1), .addr(addr), .data_in(data_in),
    .busy(busy), .byte_over(byte_over), .data_out(data_out), .txn_error(txn_error),
    .scl(iic_scl), .sda_in(sda_sync[1]), .sda_out(sda_out), .sda_out_en(sda_out_en)
);
// Open drain: a logical high releases SDA to the board's pull-up.
assign iic_sda = (sda_out_en && !sda_out) ? 1'b0 : 1'bz;
endmodule
