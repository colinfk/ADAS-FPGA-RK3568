// DDR frame writer. It accepts 128-bit words containing eight RGB565 pixels
// and emits the reference DDR controller's 16-beat write bursts. The input FIFO is the
// only backpressure point; no reader/PCIe signal is consulted here.
module ddr_frame_writer #(
    parameter integer ADDR_WIDTH  = 28,
    parameter integer FRAME_WORDS = 20'd259200,
    parameter integer ADDR_SHIFT  = 3
) (
    input                       clk,
    input                       rst_n,
    input                       i_ddr_init_done,
    input                       i_write_enable,
    input      [1:0]            i_write_frame_id,

    input                       i_stream_valid,
    output                      o_stream_ready,
    input      [127:0]          i_stream_data,
    input                       i_stream_epoch,
    input                       i_stream_sof,
    input                       i_stream_eof,
    input                       i_stream_abort,
    input      [19:0]           i_stream_word_index,

    output reg                  o_axi_awvalid,
    input                       i_axi_awready,
    output reg [ADDR_WIDTH-1:0] o_axi_awaddr,
    output reg [3:0]            o_axi_awuser_id,
    output reg                  o_axi_awuser_ap,
    output reg [3:0]            o_axi_awlen,
    input                       i_axi_wready,
    output reg [127:0]          o_axi_wdata,
    output reg [15:0]           o_axi_wstrb,
    input       [3:0]           i_axi_wuser_id,
    input                       i_axi_wlast,

    output reg                  o_frame_ready,
    output reg [1:0]            o_frame_id,
    output reg [31:0]           o_frame_seq,
    output reg                  o_frame_error,
    output reg [31:0]           o_frame_drop_count,
    output                      o_busy,
    output reg                  o_frame_abort,
    output reg [31:0]           o_geometry_abort_count,
    output [127:0]              o_fault_diag
);

localparam integer BURST_WORDS = 16;
localparam [1:0] S_FILL  = 2'd0;
localparam [1:0] S_AW    = 2'd1;
localparam [1:0] S_WRITE = 2'd2;

reg [1:0] state;
reg [3:0] fill_count;
reg [3:0] write_count;
reg [127:0] burst_data [0:BURST_WORDS-1];
reg [ADDR_WIDTH-1:0] burst_addr;
reg [1:0] burst_frame_id;
reg burst_eof;
reg [19:0] expected_word;
reg stream_epoch;
reg [31:0] first_expected, first_received, first_context;
wire [3:0] fault_reason = (i_stream_word_index != expected_word) ? 4'd1 :
    (i_stream_sof != (expected_word==0)) ? 4'd2 :
    (i_stream_eof != (expected_word==FRAME_WORDS-1)) ? 4'd3 :
    ((expected_word!=0) && (i_stream_epoch!=stream_epoch)) ? 4'd4 :
    (i_stream_eof && (fill_count!=15)) ? 4'd5 : 4'd0;
assign o_fault_diag={first_received, first_expected, first_context,32'h57524631};
integer burst_i;

wire [ADDR_WIDTH-1:0] stream_word_addr =
    {{(ADDR_WIDTH-20){1'b0}}, i_stream_word_index} << ADDR_SHIFT;
// Pango's 28-bit user address is in 16-bit units. A 4 MiB physical slot
// therefore advances by 2^21 user-address units.
wire [ADDR_WIDTH-1:0] stream_frame_base =
    {{(ADDR_WIDTH-23){1'b0}}, i_write_frame_id, 21'd0};
wire stream_accept = i_stream_valid && o_stream_ready;

assign o_stream_ready = (state == S_FILL) && i_ddr_init_done && i_write_enable && !o_frame_abort;
assign o_busy = (state != S_FILL);

always @(*) begin
    o_axi_awvalid = (state == S_AW);
    o_axi_awaddr  = burst_addr;
    o_axi_awuser_id = 4'd0;
    o_axi_awuser_ap = 1'b0;
    o_axi_awlen   = 4'd15;
    o_axi_wdata   = burst_data[write_count];
    o_axi_wstrb   = 16'hffff;
end

always @(posedge clk) begin
    if (!rst_n) begin
        state            <= S_FILL;
        fill_count       <= 4'd0;
        write_count      <= 4'd0;
        burst_addr       <= {ADDR_WIDTH{1'b0}};
        burst_frame_id   <= 2'd0;
        burst_eof        <= 1'b0;
        expected_word    <= 20'd0;
        stream_epoch<=0; first_expected<=0; first_received<=0; first_context<=0;
        o_geometry_abort_count<=0;
        o_frame_abort    <= 1'b0;
        o_frame_ready    <= 1'b0;
        o_frame_id       <= 2'd0;
        o_frame_seq      <= 32'd0;
        o_frame_error    <= 1'b0;
        o_frame_drop_count <= 32'd0;
        for (burst_i = 0; burst_i < BURST_WORDS; burst_i = burst_i + 1)
            burst_data[burst_i] <= 128'd0;
    end else begin
        o_frame_ready <= 1'b0;
        o_frame_abort <= 1'b0;

        case (state)
            S_FILL: begin
                if (stream_accept) begin
                    if (i_stream_abort) begin
                        fill_count<=0; expected_word<=0; o_frame_abort<=1;
                        o_geometry_abort_count<=o_geometry_abort_count+1'b1;
                    end else
                    // Missing FIFO words, premature EOF or a new VS cannot
                    // publish a partial/mixed frame as READY. Release it and
                    // let the boundary drain until the next SOF.
                    if (fault_reason != 0) begin
                        fill_count <= 4'd0;
                        expected_word <= 20'd0;
                        o_frame_abort <= 1'b1;
                        o_frame_error <= 1'b1;
                        o_frame_drop_count <= o_frame_drop_count + 32'd1;
                        if (!o_frame_error) begin
                            first_expected<={12'd0,expected_word};
                            first_received<={12'd0,i_stream_word_index};
                            first_context<={16'd0,fill_count,5'd0,
                                i_stream_sof,i_stream_eof,i_stream_epoch,fault_reason};
                        end
                    end else begin
                    if (expected_word==0) stream_epoch<=i_stream_epoch;
                    expected_word <= expected_word + 20'd1;
                    if (fill_count == 4'd0) begin
                        burst_addr  <= stream_frame_base + stream_word_addr;
                        burst_frame_id <= i_write_frame_id;
                    end

                    burst_data[fill_count] <= i_stream_data;

                    if (fill_count == 4'd15) begin
                        burst_eof  <= i_stream_eof;
                        fill_count <= 4'd0;
                        state      <= S_AW;
                    end else begin
                        fill_count <= fill_count + 4'd1;
                    end
                    end
                end
            end

            S_AW: begin
                if (i_axi_awready)
                    state <= S_WRITE;
            end

            S_WRITE: begin
                if (i_axi_wready) begin
                    if (write_count == 4'd15) begin
                        write_count <= 4'd0;
                        state <= S_FILL;
                        if (burst_eof) begin
                            expected_word <= 20'd0;
                            o_frame_ready <= 1'b1;
                            o_frame_id    <= burst_frame_id;
                            o_frame_seq   <= o_frame_seq + 32'd1;
                        end
                    end else begin
                        write_count <= write_count + 4'd1;
                    end
                end
            end

            default: state <= S_FILL;
        endcase
    end
end

// These fields are part of the controller contract. The generated DDR core
// has no WVALID/RREADY pins; WREADY is the write-beat accept indication.
wire unused_axi_sideband = i_stream_sof ^ i_axi_wlast ^ i_axi_wuser_id[0];

endmodule
