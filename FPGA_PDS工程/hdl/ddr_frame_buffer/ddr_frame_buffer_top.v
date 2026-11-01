// DDR frame-buffer boundary. This module owns the vendor DDR user port and
// keeps HDMI CDC, DDR burst writing, and DDR burst reading separate from the
// existing PCIe/TLP hierarchy.
module ddr_frame_buffer_top #(
    parameter integer ADDR_WIDTH  = 28,
    parameter integer FRAME_WORDS = 20'd259200,
    parameter integer FIFO_ADDR_WIDTH = 10,
    parameter integer LATEST_FRAME = 1
) (
    input                       free_clk,
    input                       rst_board,

    input                       pix_clk,
    input                       pix_rst_n,
    input                       i_frame_valid,
    input      [127:0]          i_frame_data,
    input                       i_frame_epoch,
    input                       i_frame_sof,
    input                       i_frame_eof,
    input                       i_frame_abort,
    input      [19:0]           i_frame_word_index,
    output                      o_frame_fifo_overflow,

    output                      o_ddr_init_done,
    output      [9:0]            o_ddr_init_diag,
    output                      o_ddr_error,
    output                      o_ddr_heartbeat,
    output                      o_ddr_user_clk,
    output                      o_ddr_user_rst_n,
    output                      o_frame_ready,
    output      [1:0]           o_frame_id,
    output      [31:0]          o_frame_seq,
    output                      o_frame_writer_error,
    output                      o_ownership_drop, o_writer_fault,
    output [127:0]              o_writer_diag, o_writer_fault_diag,
    output [127:0]              o_queue_diag,

    input                       i_reader_start,
    input                       i_reader_cancel,
    output                      o_reader_pending,
    output                      o_reader_busy,
    output                      o_reader_valid,
    input                       i_reader_ready,
    output      [127:0]          o_reader_data,
    output                      o_reader_epoch,
    output                      o_reader_sof,
    output                      o_reader_eof,
    output      [19:0]           o_reader_word_index,
    output                      o_reader_done,
    output                      o_reader_error,

    output                      mem_cs_n,
    output                      mem_rst_n,
    output                      mem_ck,
    output                      mem_ck_n,
    output                      mem_cke,
    output                      mem_ras_n,
    output                      mem_cas_n,
    output                      mem_we_n,
    output                      mem_odt,
    output      [14:0]           mem_a,
    output      [2:0]            mem_ba,
    inout       [1:0]            mem_dqs,
    inout       [1:0]            mem_dqs_n,
    inout       [15:0]           mem_dq,
    output      [1:0]            mem_dm
);

localparam integer FIFO_DATA_WIDTH = 152;

wire ddr_user_rst_n;
wire ddr_init_done_status;
wire [9:0] ddr_init_diag_status;
wire ddr_error_status;
wire ddr_uart_txd;
wire [7:0] ddr_error_count_status;
wire [3:0] ddr_test_state_status;
wire ddr_bist_running_status;
wire ddr_user_clk;
wire ddr_heartbeat_raw;
reg [2:0] user_reset_sync;
wire fifo_writer_priority;
reg  fifo_writer_priority_sync1;
reg  fifo_writer_priority_sync2;
wire user_rst_n = user_reset_sync[2];
always @(posedge ddr_user_clk or negedge ddr_user_rst_n) begin
    if (!ddr_user_rst_n)
        user_reset_sync <= 3'b000;
    else
        user_reset_sync <= {user_reset_sync[1:0], 1'b1};
end

always @(posedge ddr_user_clk or negedge ddr_user_rst_n) begin
    if (!ddr_user_rst_n) begin
        fifo_writer_priority_sync1 <= 1'b0;
        fifo_writer_priority_sync2 <= 1'b0;
    end else begin
        fifo_writer_priority_sync1 <= fifo_writer_priority;
        fifo_writer_priority_sync2 <= fifo_writer_priority_sync1;
    end
end

wire [FIFO_DATA_WIDTH-1:0] fifo_raw_data;
wire fifo_raw_valid;
reg [FIFO_DATA_WIDTH-1:0] fifo_rd_data;
reg fifo_rd_valid;
wire fifo_wr_ready;
wire fifo_overflow;

wire writer_awvalid;
wire writer_awready;
wire [ADDR_WIDTH-1:0] writer_awaddr;
wire [3:0] writer_awid;
wire writer_awap;
wire [3:0] writer_awlen;
wire writer_wready;
wire [127:0] writer_wdata;
wire [15:0] writer_wstrb;
wire writer_frame_ready;
wire [1:0] writer_frame_id;
wire [31:0] writer_frame_seq;
wire writer_error;
wire writer_abort;
wire [31:0] writer_drop_count;
wire [31:0] geometry_abort_count;
reg [31:0] ownership_drop_count;
wire [31:0] preview_drop_count;
assign o_queue_diag={32'd0,preview_drop_count,ownership_drop_count,32'h51535431};
assign o_writer_diag={ownership_drop_count, geometry_abort_count,writer_drop_count,32'h57525431};
wire writer_stream_ready;
wire writer_stream_enable;
wire writer_allow;
wire writer_start;
wire [1:0] writer_selected_frame;
wire writer_drop;
wire [1:0] frame0_state;
wire [1:0] frame1_state;
wire [1:0] frame2_state;
wire [1:0] frame3_state;
reg  writer_drop_seen;

wire reader_arvalid;
wire reader_arready;
wire [ADDR_WIDTH-1:0] reader_araddr;
wire [3:0] reader_arid;
wire [3:0] reader_arlen;
wire [127:0] reader_rdata;
wire reader_rvalid;
wire [3:0] reader_rid;
wire reader_rlast;

wire [127:0] reader_data;
wire reader_epoch;
wire reader_sof;
wire reader_eof;
wire [19:0] reader_word_index;
wire reader_start;
wire [1:0] reader_frame_id;
wire reader_release;
reg  reader_request_pending;
wire reader_cancel=(i_reader_cancel === 1'b1);
wire reader_request = (reader_request_pending | i_reader_start) && !reader_cancel;
assign o_reader_pending=reader_request_pending || reader_start;

wire fifo_head_sof = fifo_rd_valid && fifo_rd_data[129];
wire fifo_head_eof = fifo_rd_valid && fifo_rd_data[130];
wire ownership_frame_start = fifo_head_sof && !writer_allow && !writer_drop;
// Reject the SOF once when all four buffers are owned, then drain orphan words.
// Resynchronization depends on the next SOF, not an EOF that FIFO overflow
// might have discarded. HDMI is never held waiting for PCIe.
wire fifo_consume_ready = writer_stream_ready ||
                         (!writer_allow && (!fifo_head_sof || writer_drop));
assign writer_stream_enable = writer_allow && !writer_frame_ready && !writer_abort;
wire fifo_reset_n = pix_rst_n && ddr_user_rst_n;
reg [1:0] fifo_pix_reset, fifo_ddr_reset;
always @(posedge pix_clk or negedge fifo_reset_n)
    if (!fifo_reset_n) fifo_pix_reset <= 2'b00;
    else fifo_pix_reset <= {fifo_pix_reset[0], 1'b1};
always @(posedge ddr_user_clk or negedge fifo_reset_n)
    if (!fifo_reset_n) fifo_ddr_reset <= 2'b00;
    else fifo_ddr_reset <= {fifo_ddr_reset[0], 1'b1};

ddr_async_fifo #(
    .DATA_WIDTH (FIFO_DATA_WIDTH),
    .ADDR_WIDTH (FIFO_ADDR_WIDTH)
) u_hdmi_ddr_fifo (
    .wr_clk      (pix_clk),
    .wr_rst_n    (fifo_pix_reset[1]),
    .wr_valid    (i_frame_valid),
    .wr_data     ({i_frame_abort, i_frame_word_index, i_frame_eof, i_frame_sof,
                  i_frame_epoch, i_frame_data}),
    .wr_ready    (fifo_wr_ready),
    .wr_overflow (fifo_overflow),
    .wr_priority (fifo_writer_priority),
    .rd_clk      (ddr_user_clk),
    .rd_rst_n    (fifo_ddr_reset[1]),
    .rd_valid    (fifo_raw_valid),
    .rd_data     (fifo_raw_data),
    .rd_ready    (!fifo_rd_valid || fifo_consume_ready)
);

// Elastic register after the inferred BRAM output. This separates RAM access
// from the writer's metadata comparisons and first-fault capture enables.
// A stalled ownership/writer boundary holds the complete 152-bit word.
always @(posedge ddr_user_clk or negedge fifo_ddr_reset[1]) begin
    if (!fifo_ddr_reset[1]) begin
        fifo_rd_valid <= 1'b0;
        fifo_rd_data <= 0;
    end else if (!fifo_rd_valid || fifo_consume_ready) begin
        fifo_rd_valid <= fifo_raw_valid;
        if (fifo_raw_valid) fifo_rd_data <= fifo_raw_data;
    end
end

frame_ownership #(.LATEST_FRAME(LATEST_FRAME)) u_frame_ownership (
    .clk                    (ddr_user_clk),
    .rst_n                  (user_rst_n),
    .i_frame_start          (ownership_frame_start),
    .i_frame_write_done    (writer_frame_ready),
    .i_frame_write_abort   (writer_abort),
    .o_writer_start        (writer_start),
    .o_writer_frame        (writer_selected_frame),
    .o_writer_drop         (writer_drop),
    .o_writer_allow        (writer_allow),
    .i_reader_request      (reader_request),
    .o_reader_start        (reader_start),
    .o_reader_frame        (reader_frame_id),
    .i_reader_release      (reader_release),
    .i_reader_release_frame(reader_frame_id),
    .o_frame0_state        (frame0_state),
    .o_frame1_state        (frame1_state),
    .o_frame2_state        (frame2_state),
    .o_frame3_state        (frame3_state),
    .o_preview_drop_count  (preview_drop_count)
);

assign reader_release = o_reader_done;

always @(posedge ddr_user_clk) begin
    if (!user_rst_n || reader_cancel)
        reader_request_pending <= 1'b0;
    else if (reader_start)
        reader_request_pending <= 1'b0;
    else if (i_reader_start)
        reader_request_pending <= 1'b1;
end

always @(posedge ddr_user_clk) begin
    if (!user_rst_n) begin
        writer_drop_seen <= 1'b0;
        ownership_drop_count<=0;
    end else if (writer_drop) begin
        writer_drop_seen <= 1'b1;
        ownership_drop_count<=ownership_drop_count+1'b1;
    end
end

ddr_frame_writer #(
    .ADDR_WIDTH  (ADDR_WIDTH),
    .FRAME_WORDS (FRAME_WORDS)
) u_frame_writer (
    .clk                    (ddr_user_clk),
    .rst_n                  (user_rst_n),
    .i_ddr_init_done        (ddr_init_done_status),
    .i_write_enable         (writer_stream_enable),
    .i_write_frame_id       (writer_selected_frame),
    .i_stream_valid         (fifo_rd_valid),
    .o_stream_ready         (writer_stream_ready),
    .i_stream_data          (fifo_rd_data[127:0]),
    .i_stream_epoch         (fifo_rd_data[128]),
    .i_stream_sof           (fifo_rd_data[129]),
    .i_stream_eof           (fifo_rd_data[130]),
    .i_stream_abort         (fifo_rd_data[151]),
    .i_stream_word_index    (fifo_rd_data[150:131]),
    .o_axi_awvalid          (writer_awvalid),
    .i_axi_awready          (writer_awready),
    .o_axi_awaddr           (writer_awaddr),
    .o_axi_awuser_id        (writer_awid),
    .o_axi_awuser_ap        (writer_awap),
    .o_axi_awlen            (writer_awlen),
    .i_axi_wready           (writer_wready),
    .o_axi_wdata            (writer_wdata),
    .o_axi_wstrb            (writer_wstrb),
    .i_axi_wuser_id         (4'd0),
    .i_axi_wlast            (1'b0),
    .o_frame_ready          (writer_frame_ready),
    .o_frame_id             (writer_frame_id),
    .o_frame_seq            (writer_frame_seq),
    .o_frame_error          (writer_error),
    .o_frame_drop_count     (writer_drop_count),
    .o_busy                 (),
    .o_frame_abort          (writer_abort),
    .o_geometry_abort_count (geometry_abort_count),
    .o_fault_diag           (o_writer_fault_diag)
);

ddr_frame_reader #(
    .ADDR_WIDTH  (ADDR_WIDTH),
    .FRAME_WORDS (FRAME_WORDS)
) u_frame_reader (
    .clk                    (ddr_user_clk),
    .rst_n                  (user_rst_n),
    .i_start                (reader_start),
    .i_cancel               (reader_cancel),
    .i_frame_id             (reader_frame_id),
    .i_writer_priority      (fifo_writer_priority_sync2),
    .o_busy                 (o_reader_busy),
    .o_axi_arvalid          (reader_arvalid),
    .i_axi_arready          (reader_arready),
    .o_axi_araddr           (reader_araddr),
    .o_axi_aruser_id        (reader_arid),
    .o_axi_arlen            (reader_arlen),
    .i_axi_rdata            (reader_rdata),
    .i_axi_rvalid           (reader_rvalid),
    .i_axi_rid              (reader_rid),
    .i_axi_rlast            (reader_rlast),
    .o_stream_valid         (o_reader_valid),
    .i_stream_ready         (i_reader_ready),
    .o_stream_data          (reader_data),
    .o_stream_epoch         (reader_epoch),
    .o_stream_sof           (reader_sof),
    .o_stream_eof           (reader_eof),
    .o_stream_word_index    (reader_word_index),
    .o_frame_done           (o_reader_done),
    .o_frame_error          (o_reader_error)
);

assign o_frame_fifo_overflow = fifo_overflow;
assign o_ddr_init_done       = ddr_init_done_status;
assign o_ddr_init_diag       = ddr_init_diag_status;
assign o_ddr_heartbeat       = ddr_heartbeat_raw;
assign o_ddr_user_clk        = ddr_user_clk;
assign o_ddr_user_rst_n      = user_rst_n;
assign o_frame_ready         = writer_frame_ready;
assign o_frame_id            = writer_frame_id;
assign o_frame_seq           = writer_frame_seq;
// Surface ownership drops through the same sticky writer-status bit. A frame
// rejected because all four buffers are owned must not look like clean capture.
assign o_ownership_drop = writer_drop_seen;
assign o_writer_fault = writer_error;
assign o_frame_writer_error  = writer_error | writer_drop_seen;
assign o_reader_data         = reader_data;
assign o_reader_epoch        = reader_epoch;
assign o_reader_sof          = reader_sof;
assign o_reader_eof          = reader_eof;
assign o_reader_word_index   = reader_word_index;
assign o_ddr_error = ddr_error_status | fifo_overflow | writer_error |
                     writer_drop_seen | o_reader_error;

test_ddr #(
    .EXTERNAL_AXI_MASTER (1'b1)
) u_ddr_user_port (
    .free_clk                 (free_clk),
    .rst_board                (rst_board),
    .uart_rxd                 (1'b1),
    .uart_txd                 (ddr_uart_txd),
    .mem_cs_n                 (mem_cs_n),
    .mem_rst_n                (mem_rst_n),
    .mem_ck                   (mem_ck),
    .mem_ck_n                 (mem_ck_n),
    .mem_cke                  (mem_cke),
    .mem_ras_n                (mem_ras_n),
    .mem_cas_n                (mem_cas_n),
    .mem_we_n                 (mem_we_n),
    .mem_odt                  (mem_odt),
    .mem_a                    (mem_a),
    .mem_ba                   (mem_ba),
    .mem_dqs                  (mem_dqs),
    .mem_dqs_n                (mem_dqs_n),
    .mem_dq                   (mem_dq),
    .mem_dm                   (mem_dm),
    .heart_beat_led           (ddr_heartbeat_raw),
    .err_flag_led             (),
    .ddr_init_done_status     (ddr_init_done_status),
    .o_ddr_init_diag          (ddr_init_diag_status),
    .ddr_error_count_status   (ddr_error_count_status),
    .ddr_error_status         (ddr_error_status),
    .ddr_test_state_status    (ddr_test_state_status),
    .ddr_bist_running_status  (ddr_bist_running_status),
    .ddr_user_clk             (ddr_user_clk),
    .ddr_user_rst_n           (ddr_user_rst_n),
    .ext_axi_awaddr           (writer_awaddr),
    .ext_axi_awuser_ap        (writer_awap),
    .ext_axi_awuser_id        (writer_awid),
    .ext_axi_awlen            (writer_awlen),
    .ext_axi_awvalid          (writer_awvalid),
    .ext_axi_awready          (writer_awready),
    .ext_axi_wdata            (writer_wdata),
    .ext_axi_wstrb            (writer_wstrb),
    .ext_axi_wready           (writer_wready),
    .ext_axi_araddr           (reader_araddr),
    .ext_axi_aruser_ap        (1'b0),
    .ext_axi_aruser_id        (reader_arid),
    .ext_axi_arlen            (reader_arlen),
    .ext_axi_arvalid          (reader_arvalid),
    .ext_axi_arready          (reader_arready),
    .ext_axi_rdata            (reader_rdata),
    .ext_axi_rid              (reader_rid),
    .ext_axi_rlast            (reader_rlast),
    .ext_axi_rvalid           (reader_rvalid)
);

endmodule
