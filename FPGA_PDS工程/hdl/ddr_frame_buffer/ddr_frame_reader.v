// DDR frame reader. It issues 16-beat reads only when the local output FIFO
// has room. The consumer may stall; HDMI writes are not connected to this
// ready signal.
module ddr_frame_reader #(
    parameter integer ADDR_WIDTH  = 28,
    parameter integer FRAME_WORDS = 20'd259200,
    parameter integer ADDR_SHIFT  = 3
) (
    input                       clk,
    input                       rst_n,
    input                       i_start,
    input                       i_cancel,
    input      [1:0]            i_frame_id,
    input                       i_writer_priority,
    output                      o_busy,

    output                      o_axi_arvalid,
    input                       i_axi_arready,
    output      [ADDR_WIDTH-1:0] o_axi_araddr,
    output      [3:0]            o_axi_aruser_id,
    output      [3:0]            o_axi_arlen,
    input       [127:0]          i_axi_rdata,
    input                       i_axi_rvalid,
    input       [3:0]            i_axi_rid,
    input                       i_axi_rlast,

    output                      o_stream_valid,
    input                       i_stream_ready,
    output      [127:0]          o_stream_data,
    output                      o_stream_epoch,
    output                      o_stream_sof,
    output                      o_stream_eof,
    output      [19:0]           o_stream_word_index,
    output reg                  o_frame_done,
    output reg                  o_frame_error
);

localparam integer FIFO_DEPTH = 64;
localparam [1:0] S_IDLE  = 2'd0;
localparam [1:0] S_AR    = 2'd1;
localparam [1:0] S_DATA  = 2'd2;
localparam [1:0] S_DRAIN = 2'd3;

reg [1:0] state;
reg ar_pending;
reg cancelling;
wire cancel_requested = (i_cancel === 1'b1) || cancelling;
reg [1:0] frame_id_reg;
reg [19:0] read_word_index;
reg [19:0] stream_word_index;
reg [4:0] burst_count;
reg [127:0] data_fifo [0:FIFO_DEPTH-1];
reg [5:0] fifo_wr_ptr;
reg [5:0] fifo_rd_ptr;
reg [6:0] fifo_count;
reg output_valid;
reg [127:0] output_data;
reg [19:0] output_word_index;

// Four consecutive 4 MiB physical slots in a 16-bit-addressed DDR user map.
wire [ADDR_WIDTH-1:0] frame_base =
    {{(ADDR_WIDTH-23){1'b0}}, frame_id_reg, 21'd0};
wire [ADDR_WIDTH-1:0] read_addr = frame_base +
    ({{(ADDR_WIDTH-20){1'b0}}, read_word_index} << ADDR_SHIFT);
wire fifo_push = (state == S_DATA) && i_axi_rvalid &&
                 !cancel_requested && (fifo_count < FIFO_DEPTH);
// Register the FIFO mux before the downstream PCIe CDC write port. Hold the
// complete output word under backpressure, and retain frame ownership until
// the final registered word has actually been accepted by the consumer.
wire output_available = !output_valid || i_stream_ready;
wire fifo_pop = (fifo_count != 0) && output_available && !cancel_requested;
wire fifo_has_burst_space = (fifo_count <= FIFO_DEPTH - 16);

assign o_busy = (state != S_IDLE);
// Once presented, an address remains valid through priority/cancel changes.
assign o_axi_arvalid = (state == S_AR) && ar_pending;
assign o_axi_araddr = read_addr;
assign o_axi_aruser_id = 4'd0;
assign o_axi_arlen = 4'd15;

assign o_stream_valid = output_valid && !cancel_requested;
assign o_stream_data = output_data;
assign o_stream_epoch = frame_id_reg[0];
assign o_stream_word_index = output_word_index;
assign o_stream_sof = (output_word_index == 20'd0) && o_stream_valid;
assign o_stream_eof = (output_word_index == FRAME_WORDS - 1) && o_stream_valid;

always @(posedge clk) begin
    if (!rst_n) begin
        state          <= S_IDLE;
        ar_pending     <= 0;
        cancelling     <= 0;
        frame_id_reg   <= 2'd0;
        read_word_index <= 20'd0;
        stream_word_index <= 20'd0;
        burst_count    <= 5'd0;
        fifo_wr_ptr    <= 6'd0;
        fifo_rd_ptr    <= 6'd0;
        fifo_count     <= 7'd0;
        output_valid<=0; output_data<=0; output_word_index<=0;
        o_frame_done   <= 1'b0;
        o_frame_error  <= 1'b0;
        // Reset pointers/valid state, not the RAM payload. A word is exposed
        // only after a corresponding write incremented fifo_count. Removing
        // the array reset permits RAM inference and avoids 8192 resettable
        // flip-flops/high-fanout word enables on the native DDR response path.
    end else begin
        o_frame_done <= 1'b0;
        if (i_cancel === 1'b1) cancelling <= 1'b1;
        if (output_available) begin
            output_valid<=fifo_pop;
            if (fifo_pop) begin
                output_data<=data_fifo[fifo_rd_ptr];
                output_word_index<=stream_word_index;
            end
        end

        if (fifo_push) begin
            data_fifo[fifo_wr_ptr] <= i_axi_rdata;
            fifo_wr_ptr <= fifo_wr_ptr + 6'd1;
        end
        if (fifo_pop)
            fifo_rd_ptr <= fifo_rd_ptr + 6'd1;
        if (fifo_pop)
            stream_word_index <= stream_word_index + 20'd1;
        case ({fifo_push, fifo_pop})
            2'b10: fifo_count <= fifo_count + 7'd1;
            2'b01: fifo_count <= fifo_count - 7'd1;
            default: fifo_count <= fifo_count;
        endcase

        if (cancel_requested) begin
            output_valid <= 0; fifo_count <= 0;
            fifo_wr_ptr <= 0; fifo_rd_ptr <= 0;
        end
        case (state)
            S_IDLE: begin
                ar_pending <= 0;
                cancelling <= 0;
                if (i_start && !cancel_requested) begin
                    frame_id_reg    <= i_frame_id;
                    read_word_index <= 20'd0;
                    stream_word_index <= 20'd0;
                    burst_count     <= 5'd0;
                    state           <= S_AR;
                end
            end

            S_AR: begin
                if (!ar_pending && cancel_requested) begin
                    o_frame_done <= 1; state <= S_IDLE; cancelling <= 0;
                end else if (!ar_pending && fifo_has_burst_space && !i_writer_priority) begin
                    ar_pending <= 1;
                end
                if (o_axi_arvalid && i_axi_arready) begin
                    ar_pending <= 0;
                    burst_count <= 5'd0;
                    state <= S_DATA;
                end
            end

            S_DATA: begin
                if (i_axi_rvalid) begin
                    if (fifo_count >= FIFO_DEPTH)
                        o_frame_error <= 1'b1;
                    if (burst_count == 5'd15) begin
                        if (cancel_requested || read_word_index + 20'd16 >= FRAME_WORDS) begin
                            state <= S_DRAIN;
                        end else begin
                            read_word_index <= read_word_index + 20'd16;
                            state <= S_AR;
                        end
                    end else begin
                        burst_count <= burst_count + 5'd1;
                    end
                end
            end

            S_DRAIN: begin
                if (cancel_requested || (fifo_count == 0 && !output_valid)) begin
                    o_frame_done <= 1'b1;
                    state <= S_IDLE;
                end
            end

            default: state <= S_IDLE;
        endcase

        // The generated controller returns one response per requested beat;
        // these fields are retained as an inexpensive protocol sanity check.
        if (i_axi_rvalid && (state != S_DATA || i_axi_rid != 4'd0 ||
            (i_axi_rlast != (burst_count == 5'd15))))
            o_frame_error <= 1'b1;
    end
end

endmodule
