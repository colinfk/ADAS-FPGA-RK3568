// Four-frame ownership: FREE -> WRITING -> READY -> READING -> FREE.
// Preview reclaims obsolete READY slots; ordered mode drops when full.
// WRITING and READING slots are never reclaimed; HDMI cannot be backpressured.
module frame_ownership #(parameter LATEST_FRAME=0) (
    input           clk,
    input           rst_n,
    input           i_frame_start,
    input           i_frame_write_done,
    input           i_frame_write_abort,
    output reg      o_writer_start,
    output reg [1:0] o_writer_frame,
    output reg      o_writer_drop,
    output          o_writer_allow,
    input           i_reader_request,
    output reg      o_reader_start,
    output reg [1:0] o_reader_frame,
    input           i_reader_release,
    input      [1:0] i_reader_release_frame,
    output     [1:0] o_frame0_state,
    output     [1:0] o_frame1_state,
    output     [1:0] o_frame2_state,
    output     [1:0] o_frame3_state,
    output reg [31:0] o_preview_drop_count
);

localparam [1:0] FREE    = 2'd0;
localparam [1:0] WRITING = 2'd1;
localparam [1:0] READY   = 2'd2;
localparam [1:0] READING = 2'd3;

reg [1:0] state [0:3];
reg [31:0] ready_sequence [0:3];
reg [31:0] next_sequence;
reg [1:0] writer_cursor;
reg writer_active;
reg [1:0] writer_frame;
reg reader_active;
reg [1:0] reader_frame;
reg writer_pick_valid;
reg [1:0] writer_pick;
reg reader_pick_valid;
reg [1:0] reader_pick;
reg [1:0] latest_ready_frame;
reg [3:0] free_mask;
reg [3:0] reclaim_mask;
reg [1:0] scan_id;
reg [31:0] oldest_sequence;
reg [31:0] sequence_delta;
integer ready_i;
integer state_i;
reg [2:0] discard_ready_count;

assign o_writer_allow = writer_active;
assign o_frame0_state = state[0];
assign o_frame1_state = state[1];
assign o_frame2_state = state[2];
assign o_frame3_state = state[3];

always @(*) begin
    // Availability must not depend on the cursor/priority mux: it drives
    // register enables on the fast DDR clock. Pick priority is a separate path.
    free_mask={state[3]==FREE,state[2]==FREE,state[1]==FREE,state[0]==FREE};
    reclaim_mask={state[3]==READY && latest_ready_frame!=3,
                  state[2]==READY && latest_ready_frame!=2,
                  state[1]==READY && latest_ready_frame!=1,
                  state[0]==READY && latest_ready_frame!=0};
    if (LATEST_FRAME && i_frame_start && !writer_active && !i_reader_request)
        free_mask=free_mask | reclaim_mask;
    writer_pick_valid = |free_mask;
    writer_pick = writer_cursor;
    scan_id = writer_cursor;
    case (writer_cursor)
        0: if(free_mask[0]) writer_pick=0; else if(free_mask[1]) writer_pick=1;
           else if(free_mask[2]) writer_pick=2; else writer_pick=3;
        1: if(free_mask[1]) writer_pick=1; else if(free_mask[2]) writer_pick=2;
           else if(free_mask[3]) writer_pick=3; else writer_pick=0;
        2: if(free_mask[2]) writer_pick=2; else if(free_mask[3]) writer_pick=3;
           else if(free_mask[0]) writer_pick=0; else writer_pick=1;
        3: if(free_mask[3]) writer_pick=3; else if(free_mask[0]) writer_pick=0;
           else if(free_mask[1]) writer_pick=1; else writer_pick=2;
    endcase

    reader_pick_valid = 1'b0;
    reader_pick = 2'd0;
    oldest_sequence = 32'd0;
    sequence_delta = 32'd0;
    // One writer completes frames in order. Remember its last completed slot
    // for preview, rather than sorting four 32-bit serial numbers in one cycle.
    if (LATEST_FRAME) begin
        reader_pick=latest_ready_frame;
        reader_pick_valid=(state[latest_ready_frame]==READY);
    end else for (ready_i = 0; ready_i < 4; ready_i = ready_i + 1) begin
        scan_id = ready_i;
        if (state[scan_id] == READY) begin
            sequence_delta = ready_sequence[scan_id] - oldest_sequence;
            // At most three READY frames exist, so modulo-2^32 serial order
            // is unambiguous until more than 2^31 frames remain outstanding.
            if (!reader_pick_valid || sequence_delta[31]) begin
                reader_pick_valid = 1'b1;
                reader_pick = scan_id;
                oldest_sequence = ready_sequence[scan_id];
            end
        end
    end
    discard_ready_count=0;
    for (ready_i=0;ready_i<4;ready_i=ready_i+1)
        if (state[ready_i]==READY && ready_i!=reader_pick) discard_ready_count=discard_ready_count+1'b1;
end

always @(posedge clk) begin
    if (!rst_n) begin
        for (state_i = 0; state_i < 4; state_i = state_i + 1) begin
            state[state_i] <= FREE;
            ready_sequence[state_i] <= 32'd0;
        end
        next_sequence <= 32'd0;
        writer_cursor <= 2'd0;
        writer_active <= 1'b0;
        writer_frame <= 2'd0;
        reader_active <= 1'b0;
        reader_frame <= 2'd0;
        latest_ready_frame<=2'd0;
        o_writer_start <= 1'b0;
        o_writer_frame <= 2'd0;
        o_writer_drop <= 1'b0;
        o_reader_start <= 1'b0;
        o_reader_frame <= 2'd0;
        o_preview_drop_count<=0;
    end else begin
        o_writer_start <= 1'b0;
        o_writer_drop <= 1'b0;
        o_reader_start <= 1'b0;

        if (i_reader_release && reader_active &&
            i_reader_release_frame == reader_frame) begin
            state[reader_frame] <= FREE;
            reader_active <= 1'b0;
        end

        if ((i_frame_write_done || i_frame_write_abort) && writer_active) begin
            if (i_frame_write_abort) begin
                state[writer_frame] <= FREE;
            end else begin
                state[writer_frame] <= READY;
                ready_sequence[writer_frame] <= next_sequence;
                next_sequence <= next_sequence + 32'd1;
                latest_ready_frame<=writer_frame;
            end
            writer_active <= 1'b0;
        end

        if (LATEST_FRAME && i_frame_start && !writer_active && !i_reader_request) begin
            for (state_i=0;state_i<4;state_i=state_i+1)
                if(reclaim_mask[state_i]) state[state_i]<=FREE;
            if (|reclaim_mask)
                o_preview_drop_count<=o_preview_drop_count+
                    {2'd0,reclaim_mask[0]}+{2'd0,reclaim_mask[1]}+
                    {2'd0,reclaim_mask[2]}+{2'd0,reclaim_mask[3]};
        end
        if (i_frame_start && !writer_active) begin
            if (writer_pick_valid) begin
                state[writer_pick] <= WRITING;
                writer_active <= 1'b1;
                writer_frame <= writer_pick;
                writer_cursor <= writer_pick + 2'd1;
                o_writer_start <= 1'b1;
                o_writer_frame <= writer_pick;
            end else begin
                o_writer_drop <= 1'b1;
            end
        end

        if (i_reader_request && !reader_active && reader_pick_valid) begin
            if (LATEST_FRAME) begin
                for(state_i=0;state_i<4;state_i=state_i+1)
                    if(state[state_i]==READY && state_i!=reader_pick) state[state_i]<=FREE;
                o_preview_drop_count<=o_preview_drop_count+discard_ready_count;
            end
            state[reader_pick] <= READING;
            reader_active <= 1'b1;
            reader_frame <= reader_pick;
            o_reader_start <= 1'b1;
            o_reader_frame <= reader_pick;
        end
    end
end

endmodule
