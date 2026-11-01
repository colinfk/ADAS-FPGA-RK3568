// Append metadata to the same DDR frame as its pixels. The 256-byte trailer
// preserves 16-word DDR bursts and fits the existing RK page-aligned buffers.
module video_feature_append #(
    parameter integer IMAGE_WORDS=259200,
    parameter integer WAIT_CYCLES=4096,
    parameter integer LOW_LIGHT_PROFILE=3,
    parameter integer LOW_LIGHT_MODE=1,
    parameter integer SOURCE_KIND=0,
    parameter integer EXTENDED_ROAD_STATS=0
) (
    input clk, rst_n,
    input in_valid, in_sof, in_eof, in_abort, in_epoch,
    input [127:0] in_data,
    input [19:0] in_index,
    input [31:0] in_source_id,
    input feature_valid,
    input [511:0] feature_packet,
    input stats_valid,
    input [1023:0] stats_packet,
    output reg out_valid, out_sof, out_eof, out_abort, out_epoch,
    output reg [127:0] out_data,
    output reg [19:0] out_index,
    output [127:0] diagnostic
);
localparam IDLE=0, WAIT_PACKET=1, TRAILER=2;
reg [1:0] state;
reg [31:0] wait_count, expected_id, image_xor, completed_xor;
reg [31:0] appended_frames, errors;
reg [127:0] tail_data;
reg tail_epoch;
reg [3:0] metadata_word;
reg have_packet;
reg [511:0] latest_packet, held_packet;
reg have_stats;
reg [1023:0] latest_stats,held_stats;
wire [31:0] word_xor=in_data[31:0]^in_data[63:32]^in_data[95:64]^in_data[127:96];
localparam [7:0] TRANSPORT_VERSION=EXTENDED_ROAD_STATS ? 2 : 1;
assign diagnostic={errors,appended_frames,{TRANSPORT_VERSION,20'd0,SOURCE_KIND[0],have_packet,state},32'h46415031};
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        state<=IDLE; wait_count<=0; expected_id<=0; image_xor<=0; completed_xor<=0;
        appended_frames<=0; errors<=0; tail_data<=0; tail_epoch<=0; metadata_word<=0;
        have_packet<=0; latest_packet<=0; held_packet<=0;
        have_stats<=0;latest_stats<=0;held_stats<=0;
        out_valid<=0; out_sof<=0; out_eof<=0; out_abort<=0; out_epoch<=0;
        out_data<=0; out_index<=0;
    end else begin
        out_valid<=0; out_sof<=0; out_eof<=0; out_abort<=0;
        if(feature_valid) begin latest_packet<=feature_packet; have_packet<=1; end
        if(stats_valid) begin latest_stats<=stats_packet;have_stats<=1;end
        case(state)
            IDLE: if(in_valid) begin
                if(in_abort) begin
                    out_valid<=1; out_abort<=1; out_epoch<=in_epoch;
                    out_data<=in_data; out_index<=in_index; image_xor<=0;
                end else begin
                    image_xor<=in_sof ? word_xor : image_xor^word_xor;
                    if(in_eof) begin
                        tail_data<=in_data; tail_epoch<=in_epoch; expected_id<=in_source_id;
                        completed_xor<=in_sof ? word_xor : image_xor^word_xor;
                        wait_count<=0; state<=WAIT_PACKET;
                    end else begin
                        out_valid<=1; out_data<=in_data; out_index<=in_index;
                        out_sof<=in_sof; out_epoch<=in_epoch;
                    end
                end
            end
            WAIT_PACKET: begin
                if(in_valid || wait_count==WAIT_CYCLES-1) begin
                    out_valid<=1; out_abort<=1; out_epoch<=tail_epoch;
                    out_index<=IMAGE_WORDS-1; state<=IDLE; have_packet<=0;have_stats<=0;
                    errors<=errors+1'b1;
                end else if(have_packet && latest_packet[63:32]==expected_id &&
                            latest_packet[31:0]==32'h54414546 && latest_packet[107] &&
                            (!EXTENDED_ROAD_STATS || (have_stats && latest_stats[31:0]==32'h31584652 &&
                             latest_stats[63:32]==expected_id && latest_stats[96]))) begin
                    held_packet<=latest_packet; have_packet<=0;
                    held_stats<=latest_stats;have_stats<=0;
                    out_valid<=1; out_data<=tail_data; out_epoch<=tail_epoch;
                    out_index<=IMAGE_WORDS-1; out_sof<=(IMAGE_WORDS==1);
                    metadata_word<=0; state<=TRAILER;
                end else wait_count<=wait_count+1'b1;
            end
            TRAILER: begin
                out_valid<=1; out_epoch<=tail_epoch;
                out_index<=IMAGE_WORDS+metadata_word; out_eof<=(metadata_word==15);
                if(metadata_word<4) out_data<=held_packet[metadata_word*128 +: 128];
                else if(metadata_word==4)
                    out_data<={(EXTENDED_ROAD_STATS ? 32'h00020100 : 32'h00010100),completed_xor,expected_id,32'h44495646};
                else if(metadata_word==5)
                    out_data<={(LOW_LIGHT_PROFILE|(SOURCE_KIND<<16)),LOW_LIGHT_MODE[31:0],32'd1,32'h4658554c};
                else if(EXTENDED_ROAD_STATS && metadata_word>=6 && metadata_word<14)
                    out_data<=held_stats[(metadata_word-6)*128+:128];
                else out_data<=0;
                if(in_valid) begin
                    out_abort<=1; out_eof<=0; state<=IDLE; errors<=errors+1'b1;
                end else if(metadata_word==15) begin
                    state<=IDLE; appended_frames<=appended_frames+1'b1;
                end else metadata_word<=metadata_word+1'b1;
            end
            default: state<=IDLE;
        endcase
    end
end
endmodule
