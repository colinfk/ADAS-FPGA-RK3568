// Small dual-clock FIFO used only at the HDMI-to-DDR boundary.
// The write side never waits for the DDR reader; overflow is latched for
// software/bring-up status instead.
module ddr_async_fifo #(
    parameter integer DATA_WIDTH = 150,
    parameter integer ADDR_WIDTH = 10
) (
    input                       wr_clk,
    input                       wr_rst_n,
    input                       wr_valid,
    input      [DATA_WIDTH-1:0] wr_data,
    output                      wr_ready,
    output reg                  wr_overflow,
    output reg                  wr_priority,

    input                       rd_clk,
    input                       rd_rst_n,
    output reg                  rd_valid,
    output reg [DATA_WIDTH-1:0] rd_data,
    input                       rd_ready
);

localparam integer FIFO_DEPTH = (1 << ADDR_WIDTH);
localparam integer PTR_WIDTH  = ADDR_WIDTH + 1;

reg [DATA_WIDTH-1:0] mem [0:FIFO_DEPTH-1];
reg [PTR_WIDTH-1:0] wr_bin;
reg [PTR_WIDTH-1:0] wr_gray;
reg [PTR_WIDTH-1:0] rd_bin;
reg [PTR_WIDTH-1:0] rd_gray;
reg [PTR_WIDTH-1:0] rd_gray_wr_d1;
reg [PTR_WIDTH-1:0] rd_gray_wr_d2;
reg [PTR_WIDTH-1:0] wr_gray_rd_d1;
reg [PTR_WIDTH-1:0] wr_gray_rd_d2;
reg                 wr_full;
reg                 rd_empty;

wire wr_fire = wr_valid && wr_ready;
wire rd_fire = rd_valid && rd_ready;
wire [PTR_WIDTH-1:0] wr_bin_next = wr_bin + wr_fire;
wire [PTR_WIDTH-1:0] rd_bin_next = rd_bin + rd_fire;

function [PTR_WIDTH-1:0] bin_to_gray;
    input [PTR_WIDTH-1:0] value;
    begin
        bin_to_gray = (value >> 1) ^ value;
    end
endfunction

function [PTR_WIDTH-1:0] gray_to_bin;
    input [PTR_WIDTH-1:0] value;
    integer bit_index;
    begin
        gray_to_bin[PTR_WIDTH-1] = value[PTR_WIDTH-1];
        for (bit_index = PTR_WIDTH-2; bit_index >= 0; bit_index = bit_index-1)
            gray_to_bin[bit_index] = gray_to_bin[bit_index+1] ^ value[bit_index];
    end
endfunction

wire [PTR_WIDTH-1:0] wr_gray_next = bin_to_gray(wr_bin_next);
wire [PTR_WIDTH-1:0] rd_gray_next = bin_to_gray(rd_bin_next);
wire [PTR_WIDTH-1:0] rd_gray_full_cmp = {
    ~rd_gray_wr_d2[PTR_WIDTH-1:PTR_WIDTH-2],
    rd_gray_wr_d2[PTR_WIDTH-3:0]
};
wire [PTR_WIDTH-1:0] wr_occupancy = wr_bin - gray_to_bin(rd_gray_wr_d2);

assign wr_ready = ~wr_full;

always @(posedge wr_clk or negedge wr_rst_n) begin
    if (!wr_rst_n) begin
        rd_gray_wr_d1 <= {PTR_WIDTH{1'b0}};
        rd_gray_wr_d2 <= {PTR_WIDTH{1'b0}};
    end else begin
        rd_gray_wr_d1 <= rd_gray;
        rd_gray_wr_d2 <= rd_gray_wr_d1;
    end
end

always @(posedge wr_clk or negedge wr_rst_n) begin
    if (!wr_rst_n) begin
        wr_bin      <= {PTR_WIDTH{1'b0}};
        wr_gray     <= {PTR_WIDTH{1'b0}};
        wr_full     <= 1'b0;
        wr_overflow <= 1'b0;
        wr_priority <= 1'b0;
    end else begin
        if (wr_fire)
            mem[wr_bin[ADDR_WIDTH-1:0]] <= wr_data;
        if (wr_valid && !wr_ready)
            wr_overflow <= 1'b1;
        wr_bin  <= wr_bin_next;
        wr_gray <= wr_gray_next;
        wr_full <= (wr_gray_next == rd_gray_full_cmp);
        // Hysteresis: pause new lower-priority DDR reads above 3/4 full;
        // resume only after HDMI has drained below 1/2 full.
        if (wr_occupancy >= (FIFO_DEPTH * 3 / 4))
            wr_priority <= 1'b1;
        else if (wr_occupancy <= (FIFO_DEPTH / 2))
            wr_priority <= 1'b0;
    end
end

always @(posedge rd_clk or negedge rd_rst_n) begin
    if (!rd_rst_n) begin
        wr_gray_rd_d1 <= {PTR_WIDTH{1'b0}};
        wr_gray_rd_d2 <= {PTR_WIDTH{1'b0}};
    end else begin
        wr_gray_rd_d1 <= wr_gray;
        wr_gray_rd_d2 <= wr_gray_rd_d1;
    end
end

always @(posedge rd_clk or negedge rd_rst_n) begin
    if (!rd_rst_n) begin
        rd_bin   <= {PTR_WIDTH{1'b0}};
        rd_gray  <= {PTR_WIDTH{1'b0}};
        rd_empty <= 1'b1;
        rd_valid <= 1'b0;
        rd_data  <= {DATA_WIDTH{1'b0}};
    end else begin
        if (rd_fire) begin
            rd_bin  <= rd_bin_next;
            rd_gray <= rd_gray_next;
            if (rd_gray_next == wr_gray_rd_d2) begin
                rd_empty <= 1'b1;
                rd_valid <= 1'b0;
            end else begin
                rd_empty <= 1'b0;
                rd_data  <= mem[rd_bin_next[ADDR_WIDTH-1:0]];
                rd_valid <= 1'b1;
            end
        end else if (!rd_valid && !rd_empty) begin
            rd_data  <= mem[rd_bin[ADDR_WIDTH-1:0]];
            rd_valid <= 1'b1;
        end

        if (!rd_fire && (rd_gray_next == wr_gray_rd_d2))
            rd_empty <= 1'b1;
        else if (rd_gray_next != wr_gray_rd_d2)
            rd_empty <= 1'b0;
    end
end

endmodule
