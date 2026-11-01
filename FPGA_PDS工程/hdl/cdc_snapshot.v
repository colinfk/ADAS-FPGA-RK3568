// Bundled-data CDC. The source holds the entire payload until the destination
// acknowledges it by removing request. Only request/ack cross via synchronizers.
module cdc_snapshot #(parameter WIDTH=128) (
    input src_clk, src_rst_n, input [WIDTH-1:0] src_data,
    input dst_clk, dst_rst_n, output reg [WIDTH-1:0] dst_data
);
reg request, acknowledge;
reg req_sync1, req_sync2, ack_sync1, ack_sync2;
reg [WIDTH-1:0] held_data;
// Reset either endpoint terminates the handshake at both endpoints. In
// particular, a source reset cannot leave a stale high ACK at the receiver.
wire link_rst_n = src_rst_n && dst_rst_n;
reg [1:0] src_reset_pipe, dst_reset_pipe;
always @(posedge src_clk or negedge link_rst_n)
    if (!link_rst_n) src_reset_pipe <= 0;
    else src_reset_pipe <= {src_reset_pipe[0],1'b1};
always @(posedge dst_clk or negedge link_rst_n)
    if (!link_rst_n) dst_reset_pipe <= 0;
    else dst_reset_pipe <= {dst_reset_pipe[0],1'b1};
wire src_local_rst_n=src_reset_pipe[1];
wire dst_local_rst_n=dst_reset_pipe[1];
always @(posedge src_clk or negedge src_local_rst_n) begin
    if (!src_local_rst_n) begin
        req_sync1<=0; req_sync2<=0; acknowledge<=0; held_data<=0;
    end else if (src_reset_pipe[1]) begin
        req_sync1<=request; req_sync2<=req_sync1;
        if (req_sync2 && !acknowledge) begin
            held_data<=src_data; acknowledge<=1;
        end else if (!req_sync2) acknowledge<=0;
    end
end
always @(posedge dst_clk or negedge dst_local_rst_n) begin
    if (!dst_local_rst_n) begin
        request<=0; ack_sync1<=0; ack_sync2<=0; dst_data<=0;
    end else if (dst_reset_pipe[1]) begin
        ack_sync1<=acknowledge; ack_sync2<=ack_sync1;
        if (!request && !ack_sync2) request<=1;
        else if (request && ack_sync2) begin
            dst_data<=held_data; request<=0;
        end
    end
end
endmodule
