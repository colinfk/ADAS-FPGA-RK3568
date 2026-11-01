module video_dma_scheduler #(
    parameter integer SLOT_COUNT   = 15,
    parameter integer FRAME_CHUNKS = 2025,
    parameter integer LAST_CHUNK_DWORDS = 1024,
    parameter integer FRAME_BUFFER_SOURCE = 0
) (
    input               clk,
    input               rst_n,
    input               i_cfg_wr_en,
    input       [11:0]  i_cfg_wr_addr,
    input       [127:0] i_cfg_wr_data,
    input       [15:0]  i_cfg_wr_byte_en,
    input       [SLOT_COUNT-1:0] i_chunk_ready,
    input               i_chunk_overflow,
    input               i_frame_epoch,
    input               i_source_idle,
    input               i_mwr64_req_ack,
    input               i_mwr_tx_busy,
    output reg          o_mwr64_req,
    output wire         o_mwr32_req,
    output wire [9:0]   o_req_length,
    output wire [63:0]  o_req_addr,
    output wire [31:0]  o_req_data,
    output wire [11:0]  o_video_source_base,
    output reg          o_chunk_claim,
    output reg  [3:0]   o_chunk_claim_slot,
    output reg          o_chunk_consume,
    output reg  [3:0]   o_chunk_consume_slot,
    output reg          o_frame_done,
    output reg  [31:0]  o_frame_seq,
    output reg          o_buffer_select,
    output reg          o_overflow,
    output reg          o_video_rearm,
    output reg          o_source_frame_request,
    output reg  [31:0]  o_dropped_frame_count,
    output wire         o_source_run,
    output reg [31:0]   o_command_seq,
    output wire         o_stopping,
    output wire         o_active
);

reg [63:0] frame_base0;
reg [63:0] frame_base1;
reg        video_enable;
reg        stopping;
wire source_idle=(i_source_idle !== 1'b0);
reg        req_pending;
reg        transaction_started;
reg        busy_seen;
reg [11:0] chunk_index;
reg [3:0]  chunk_slot;
reg [63:0] target_addr;
reg        next_buffer;
reg        wait_frame_start;
reg        frame_invalid;
reg        active_epoch;
reg        start_epoch;
reg [63:0] completion_addr;
reg        completion_pending;
reg        transaction_completion;

wire cfg_full  = &i_cfg_wr_byte_en;
wire cfg_base0_l = i_cfg_wr_en && cfg_full && (i_cfg_wr_addr == 12'h180);
wire cfg_base0_h = i_cfg_wr_en && cfg_full && (i_cfg_wr_addr == 12'h190);
wire cfg_base1_l = i_cfg_wr_en && cfg_full && (i_cfg_wr_addr == 12'h1a0);
wire cfg_base1_h = i_cfg_wr_en && cfg_full && (i_cfg_wr_addr == 12'h1b0);
wire cfg_ctrl    = i_cfg_wr_en && cfg_full && (i_cfg_wr_addr == 12'h1c0);
wire cfg_completion_l = i_cfg_wr_en && cfg_full && (i_cfg_wr_addr == 12'h1d0);
wire cfg_completion_h = i_cfg_wr_en && cfg_full && (i_cfg_wr_addr == 12'h1e0);
wire start_pulse = cfg_ctrl && i_cfg_wr_data[0] && !video_enable &&
                   (!FRAME_BUFFER_SOURCE || source_idle);
wire stop_pulse  = cfg_ctrl && !i_cfg_wr_data[0];
assign o_active = video_enable;
assign o_source_run=video_enable && !stopping;
assign o_stopping=stopping;

assign o_req_length        = completion_pending ? 10'd1 :
                              ((FRAME_BUFFER_SOURCE &&
                                (chunk_index == FRAME_CHUNKS - 1)) ?
                               LAST_CHUNK_DWORDS[9:0] : 10'd0);
assign o_mwr32_req         = 1'b0;
assign o_req_addr          = target_addr;
assign o_req_data          = {1'b1, next_buffer, (o_frame_seq[29:0] + 30'd1)};
assign o_video_source_base = {chunk_slot, 8'd0};

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        frame_base0           <= 64'd0;
        frame_base1           <= 64'd0;
        video_enable          <= 1'b0;
        stopping              <= 1'b0;
        o_command_seq         <= 0;
        req_pending           <= 1'b0;
        transaction_started   <= 1'b0;
        busy_seen             <= 1'b0;
        chunk_index           <= 12'd0;
        chunk_slot            <= 4'd0;
        target_addr           <= 64'd0;
        next_buffer           <= 1'b0;
        wait_frame_start      <= 1'b0;
        frame_invalid         <= 1'b0;
        active_epoch          <= 1'b0;
        start_epoch           <= 1'b0;
        completion_addr       <= 64'd0;
        completion_pending    <= 1'b0;
        transaction_completion <= 1'b0;
        o_mwr64_req           <= 1'b0;
        o_chunk_claim         <= 1'b0;
        o_chunk_claim_slot    <= 4'd0;
        o_chunk_consume       <= 1'b0;
        o_chunk_consume_slot  <= 4'd0;
        o_frame_done          <= 1'b0;
        o_frame_seq           <= 32'd0;
        o_buffer_select       <= 1'b0;
        o_overflow            <= 1'b0;
        o_video_rearm         <= 1'b0;
        o_source_frame_request <= 1'b0;
        o_dropped_frame_count <= 32'd0;
    end else begin
        o_chunk_claim   <= 1'b0;
        o_chunk_consume <= 1'b0;
        o_frame_done    <= 1'b0;
        o_video_rearm   <= 1'b0;
        o_source_frame_request <= 1'b0;

        if(cfg_ctrl) o_command_seq<=o_command_seq+1'b1;
        if (cfg_base0_l && !video_enable) frame_base0[31:0]  <= i_cfg_wr_data[31:0];
        if (cfg_base0_h && !video_enable) frame_base0[63:32] <= i_cfg_wr_data[31:0];
        if (cfg_base1_l && !video_enable) frame_base1[31:0]  <= i_cfg_wr_data[31:0];
        if (cfg_base1_h && !video_enable) frame_base1[63:32] <= i_cfg_wr_data[31:0];
        if (cfg_completion_l && !video_enable) completion_addr[31:0] <= i_cfg_wr_data[31:0];
        if (cfg_completion_h && !video_enable) completion_addr[63:32] <= i_cfg_wr_data[31:0];

        if (start_pulse) begin
            video_enable        <= 1'b1;
            stopping            <= 0;
            wait_frame_start    <= FRAME_BUFFER_SOURCE ? 1'b0 : 1'b1;
            start_epoch         <= i_frame_epoch;
            active_epoch        <= i_frame_epoch;
            frame_invalid       <= 1'b0;
            req_pending         <= 1'b0;
            transaction_started <= 1'b0;
            busy_seen           <= 1'b0;
            chunk_index         <= 12'd0;
            chunk_slot          <= 4'd0;
            // DDR mode grants exactly one frame per host START. The host
            // copies the completed buffer before granting another frame.
            // Preserve sequence and alternation across process restarts.
            if (!FRAME_BUFFER_SOURCE) begin
                next_buffer     <= 1'b0;
                o_buffer_select <= 1'b0;
                o_frame_seq     <= 32'd0;
            end
            target_addr         <= (FRAME_BUFFER_SOURCE && next_buffer) ?
                                   frame_base1 : frame_base0;
            o_overflow          <= 1'b0;
            o_mwr64_req         <= 1'b0;
            completion_pending  <= 1'b0;
            o_video_rearm       <= 1'b1;
            if (FRAME_BUFFER_SOURCE)
                o_source_frame_request <= 1'b1;
        end else if (stop_pulse && !FRAME_BUFFER_SOURCE) begin
            video_enable        <= 1'b0;
            wait_frame_start    <= 1'b0;
            frame_invalid       <= 1'b0;
            req_pending         <= 1'b0;
            transaction_started <= 1'b0;
            busy_seen           <= 1'b0;
            o_mwr64_req         <= 1'b0;
            o_video_rearm       <= 1'b1;
        end else begin
            if(FRAME_BUFFER_SOURCE && stop_pulse && video_enable) stopping<=1;
            // Retain a presented MWr until acknowledged, then drain its entire
            // transaction. Stop does not discard a committed completion.
            if(FRAME_BUFFER_SOURCE && stopping && !completion_pending &&
               !req_pending && !transaction_started && !i_mwr_tx_busy && source_idle) begin
                // Publish an ordered completion even for a cancelled grant.
                // Host STOP consumes it without exposing the partial frame;
                // a BAR read alone cannot fence earlier posted DMA writes.
                completion_pending<=1;target_addr<=completion_addr;
                frame_invalid<=0;chunk_index<=0;chunk_slot<=0;
            end
            if (!FRAME_BUFFER_SOURCE && video_enable && wait_frame_start &&
                (i_frame_epoch != start_epoch)) begin
                wait_frame_start <= 1'b0;
                active_epoch     <= i_frame_epoch;
                frame_invalid    <= 1'b0;
                o_overflow        <= 1'b0;
                chunk_index       <= 12'd0;
                chunk_slot        <= 4'd0;
                target_addr       <= next_buffer ? frame_base1 : frame_base0;
            end else if (!FRAME_BUFFER_SOURCE && video_enable &&
                         !wait_frame_start && frame_invalid &&
                         !req_pending && !transaction_started &&
                         (i_frame_epoch != active_epoch)) begin
                wait_frame_start <= 1'b0;
                active_epoch     <= i_frame_epoch;
                frame_invalid    <= 1'b0;
                o_overflow        <= 1'b0;
                chunk_index       <= 12'd0;
                chunk_slot        <= 4'd0;
                target_addr       <= next_buffer ? frame_base1 : frame_base0;
            end

            if (video_enable && !wait_frame_start && !frame_invalid &&
                (i_chunk_overflow ||
                 (!FRAME_BUFFER_SOURCE && (i_frame_epoch != active_epoch)))) begin
                frame_invalid         <= 1'b1;
                o_overflow             <= 1'b1;
                o_dropped_frame_count <= o_dropped_frame_count + 32'd1;
            end

            if (video_enable && completion_pending &&
                !req_pending && !transaction_started && !i_mwr_tx_busy) begin
                req_pending <= 1'b1;
                o_mwr64_req <= 1'b1;
            end else if (video_enable && !wait_frame_start && !frame_invalid &&
                !stopping && !stop_pulse &&
                !req_pending && !transaction_started && !i_mwr_tx_busy &&
                i_chunk_ready[chunk_slot]) begin
                req_pending <= 1'b1;
                o_mwr64_req <= 1'b1;
            end

            if (o_mwr64_req && i_mwr64_req_ack) begin
                o_mwr64_req         <= 1'b0;
                req_pending         <= 1'b0;
                transaction_started <= 1'b1;
                transaction_completion <= completion_pending;
                if (!completion_pending) begin
                    o_chunk_claim       <= 1'b1;
                    o_chunk_claim_slot  <= chunk_slot;
                end
            end

            if (transaction_started && i_mwr_tx_busy)
                busy_seen <= 1'b1;

            if (transaction_started && busy_seen && !i_mwr_tx_busy) begin
                transaction_started  <= 1'b0;
                busy_seen             <= 1'b0;
                if (transaction_completion) begin
                    completion_pending <= 1'b0;
                    transaction_completion <= 1'b0;
                    o_buffer_select <= next_buffer;
                    next_buffer <= ~next_buffer;
                    o_frame_seq <= o_frame_seq + 32'd1;
                    o_frame_done <= 1'b1;
                    if (FRAME_BUFFER_SOURCE) begin
                        video_enable <= 1'b0; stopping<=0;o_video_rearm<=1;
                    end
                    wait_frame_start <= FRAME_BUFFER_SOURCE ? 1'b0 : 1'b1;
                    target_addr <= next_buffer ? frame_base0 : frame_base1;
                end else begin
                o_chunk_consume      <= 1'b1;
                o_chunk_consume_slot <= chunk_slot;

                if (chunk_index == FRAME_CHUNKS - 1) begin
                    chunk_index <= 12'd0;
                    chunk_slot  <= 4'd0;
                    if (frame_invalid || i_chunk_overflow ||
                        (!FRAME_BUFFER_SOURCE &&
                         (i_frame_epoch != active_epoch))) begin
                        target_addr <= next_buffer ? frame_base1 : frame_base0;
                    end else begin
                        if (FRAME_BUFFER_SOURCE) begin
                            completion_pending <= 1'b1;
                            target_addr <= completion_addr;
                        end else begin
                            o_buffer_select <= next_buffer;
                            next_buffer <= ~next_buffer;
                            o_frame_seq <= o_frame_seq + 32'd1;
                            o_frame_done <= 1'b1;
                            wait_frame_start <= 1'b1;
                            start_epoch <= active_epoch;
                            target_addr <= next_buffer ? frame_base0 : frame_base1;
                        end
                    end
                end else if(!FRAME_BUFFER_SOURCE || (!stopping && !stop_pulse)) begin
                    chunk_index <= chunk_index + 12'd1;
                    if (chunk_slot == SLOT_COUNT - 1)
                        chunk_slot <= 4'd0;
                    else
                        chunk_slot <= chunk_slot + 4'd1;
                    target_addr <= target_addr + 64'd4096;
                end
                end
            end
        end
    end
end

endmodule
