// Two complete ROI pairs, 8 KB total. Ownership crosses by held toggles.
// Producer never overwrites a bank until consumer releases it. Metadata stays
// fixed for the entire outstanding request; consumer reads after 2 FF settling.
module lane_pair_buffer(
 input video_clk,video_rst_n,frame_start,input [31:0] video_frame_id,
 input left_valid,right_valid,input [10:0] left_addr,right_addr,
 input [7:0] left_gray,right_gray,
 output reg [31:0] dropped_frames,
 input cnn_clk,cnn_rst_n,output pair_valid,input pair_take,
 output reg pair_bank,output [31:0] pair_frame_id,pair_drop_count,
 input pair_release,input read_side,input [10:0] read_addr,output reg [7:0] read_data
);
reg [7:0] ram[0:8191];
reg [1:0] request,ack;
reg [1:0] ack_meta,ack_sync,req_meta,req_sync;
reg [31:0] ids[0:1],drops[0:1];
reg capture,write_bank,pair_active,preferred_bank;
reg [11:0] left_count,right_count;
wire left_accept=capture && left_valid && !frame_start;
wire right_accept=capture && right_valid && !frame_start;
wire malformed=capture && ((left_valid && left_addr!=left_count) ||
                           (right_valid && right_addr!=right_count) || (left_valid && right_valid));
wire complete_pair=capture && (left_count+(left_accept?1:0)==2048) && (right_count+(right_accept?1:0)==2048);
assign pair_valid=!pair_active && ((req_sync[0]!=ack[0]) || (req_sync[1]!=ack[1]));
assign pair_frame_id=ids[pair_bank];
assign pair_drop_count=drops[pair_bank];
wire selected_bank=(req_sync[preferred_bank]!=ack[preferred_bank]) ? preferred_bank : ~preferred_bank;
always @(posedge video_clk) begin
 if(left_accept)ram[{write_bank,1'b0,left_addr}]<=left_gray;
 else if(right_accept)ram[{write_bank,1'b1,right_addr}]<=right_gray;
 if(!video_rst_n) begin
  request<=0;ack_meta<=0;ack_sync<=0;capture<=0;write_bank<=0;
  left_count<=0;right_count<=0;dropped_frames<=0;
  ids[0]<=0;ids[1]<=0;drops[0]<=0;drops[1]<=0;
 end else begin
  ack_meta<=ack;ack_sync<=ack_meta;
  if(frame_start) begin
   left_count<=0;right_count<=0;capture<=0;
   if(capture)dropped_frames<=dropped_frames+1'b1;
   if(request[0]==ack_sync[0])begin
    write_bank<=0;capture<=1;ids[0]<=video_frame_id;drops[0]<=dropped_frames+(capture?1:0);
   end else if(request[1]==ack_sync[1])begin
    write_bank<=1;capture<=1;ids[1]<=video_frame_id;drops[1]<=dropped_frames+(capture?1:0);
   end else dropped_frames<=dropped_frames+1'b1;
  end else begin
   if(left_accept)left_count<=left_count+1'b1;
   if(right_accept)right_count<=right_count+1'b1;
   if(malformed)begin capture<=0;dropped_frames<=dropped_frames+1'b1;end
   else if(complete_pair)begin request[write_bank]<=~request[write_bank];capture<=0;end
  end
 end
end
always @(posedge cnn_clk)begin
 read_data<=ram[{pair_bank,read_side,read_addr}];
 if(!cnn_rst_n)begin ack<=0;req_meta<=0;req_sync<=0;pair_bank<=0;pair_active<=0;preferred_bank<=0;end
 else begin
  req_meta<=request;req_sync<=req_meta;
  if(pair_valid && pair_take)begin
   // With one producer and two owned slots, prefer the other slot after each
   // take. A freshly refilled slot cannot overtake the older waiting slot.
   // Only synchronized toggles enter this decision; held IDs are payload.
   pair_bank<=selected_bank;preferred_bank<=~selected_bank;
   pair_active<=1;
  end
  if(pair_release && pair_active)begin ack[pair_bank]<=req_sync[pair_bank];pair_active<=0;end
 end
end
endmodule
