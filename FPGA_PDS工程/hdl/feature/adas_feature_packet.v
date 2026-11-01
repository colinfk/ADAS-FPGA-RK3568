// Fixed-size feature packet. Coordinates are pixel coordinates in the input
// frame. The packet is deliberately small so it can later be put in a BRAM,
// DDR slot, BAR register window, or a PCIe DMA completion record.
module adas_feature_packet (
    input  [31:0]  frame_id,
    input  [15:0]  image_width,
    input  [15:0]  image_height,
    input  [7:0]   version,
    input  [7:0]   flags,
    input  [127:0] left_record,
    input  [127:0] right_record,
    input  [127:0] zebra_record,
    output reg [511:0] packet
);
    always @(*) begin
        packet = 512'd0;
        // Little-endian byte order in memory must read ASCII "FEAT".
        packet[31:0]    = 32'h54414546;
        packet[63:32]   = frame_id;
        packet[79:64]   = image_width;
        packet[95:80]   = image_height;
        packet[103:96]  = version;
        packet[111:104] = flags;
        packet[127:112] = 16'd64;       // packet length in bytes
        packet[255:128] = left_record;
        packet[383:256] = right_record;
        packet[511:384] = zebra_record;
    end
endmodule
