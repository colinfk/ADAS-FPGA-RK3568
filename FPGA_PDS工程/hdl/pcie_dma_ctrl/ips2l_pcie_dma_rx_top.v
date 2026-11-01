//////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2019 PANGO MICROSYSTEMS, INC
// ALL RIGHTS REVERVED.
//
// THE SOURCE CODE CONTAINED HEREIN IS PROPRIETARY TO PANGO MICROSYSTEMS, INC.
// IT SHALL NOT BE REPRODUCED OR DISCLOSED IN WHOLE OR IN PART OR USED BY
// PARTIES WITHOUT WRITTEN AUTHORIZATION FROM THE OWNER.
//
//////////////////////////////////////////////////////////////////////////////
//
// Library:
// Filename:ips2l_pcie_dma_rx_top.v
//////////////////////////////////////////////////////////////////////////////
module ips2l_pcie_dma_rx_top #(
    parameter                           DEVICE_TYPE = 3'd0      ,   //3'd0:EP,3'd1:Legacy EP,3'd4:RC
    parameter                           ADDR_WIDTH  = 4'd9,
    parameter                           LOSSLESS_VIDEO_SOURCE = 0,
    parameter LANE_STATUS = 0,
    parameter RX_CONTROL = 0
)(
    input                               clk                     ,   //gen1:62.5MHz,gen2:125MHz
    input                               rst_n                   ,
    input                               i_video_wr_clk          ,
    input                               i_video_wr_rst_n        ,
    input                               i_video_wr_en           ,
    input           [ADDR_WIDTH-1:0]    i_video_wr_addr         ,
    input           [127:0]             i_video_wr_data         ,
    input                               i_video_frame_epoch     ,
    input                               i_video_wr_frame_last   ,
    output  wire                        o_video_wr_ready         ,
    input           [2:0]               i_cfg_max_rd_req_size   ,
    //**********************************************************************
    //AXIS master interface
    input                               i_axis_master_tvld      ,
    output  wire                        o_axis_master_trdy      ,
    input           [127:0]             i_axis_master_tdata     ,
    input           [3:0]               i_axis_master_tkeep     ,
    input                               i_axis_master_tlast     ,
    input           [7:0]               i_axis_master_tuser     ,
    output  wire    [2:0]               o_trgt1_radm_pkt_halt   ,
//    input           [5:0]               i_radm_grant_tlp_type   ,
    //**********************************************************************
    //bar0 rd interface
    input                               i_bar0_rd_clk_en        ,
    input           [ADDR_WIDTH-1:0]    i_bar0_rd_addr          ,
    output  wire    [127:0]             o_bar0_rd_data          ,

    //bar2 rd interface
    input                               i_bar2_rd_clk_en        ,
    input           [ADDR_WIDTH-1:0]    i_bar2_rd_addr          ,
    output  wire    [127:0]             o_bar2_rd_data          ,
    //video MWr source shares the BAR2 read timing and uses a separate RAM
    output  wire    [127:0]             o_video_dma_rd_data     ,
    output  wire    [14:0]              o_video_chunk_ready     ,
    output  wire                        o_video_frame_epoch     ,
    output  wire                        o_video_frame_error     ,
    input                               i_video_chunk_claim     ,
    input           [3:0]               i_video_chunk_claim_slot,
    input                               i_video_rearm           ,
    input                               i_video_chunk_consume   ,
    input           [3:0]               i_video_chunk_slot      ,
    output  wire                        o_video_chunk_overflow  ,
    input                               i_video_status_wr_en    ,
    input           [127:0]             i_video_status_wr_data  ,
    input           [11:0]              i_video_status_wr_addr,
    output                              o_video_status_wr_ready,

    //bar1 wr interface
    output  reg                         o_bar1_wr_en            ,
    output  reg     [ADDR_WIDTH-1:0]    o_bar1_wr_addr          ,
    output  reg     [127:0]             o_bar1_wr_data          ,
    output  reg     [15:0]              o_bar1_wr_byte_en       ,
    //**********************************************************************
    //to tx top
    //req rcv
    output  wire    [2:0]               o_mrd_tc                ,
    output  wire    [2:0]               o_mrd_attr              ,
    output  wire    [9:0]               o_mrd_length            ,
    output  wire    [15:0]              o_mrd_id                ,
    output  wire    [7:0]               o_mrd_tag               ,
    output  wire    [63:0]              o_mrd_addr              ,

    output  wire                        o_cpld_req_vld          ,
    input                               i_cpld_req_rdy          ,
    input                               i_cpld_tx_rdy           ,
    //cpld rcv
    output wire                         o_cpld_rcv              ,
    output wire     [7:0]               o_cpld_tag              ,
    input                               i_tag_full              ,
    //rst tlp cnt
    output wire     [63:0]              o_dma_check_result      ,
    input                               i_tx_restart

    //output  wire    [42:0]              o_dbg_bus_rx_ctrl       ,
    //output  wire    [69:0]              o_dbg_tlp_rcv_cnt
);

wire                        mwr_wr_start;
wire    [9:0]               mwr_length;
wire    [7:0]               mwr_dwbe;
wire    [127:0]             mwr_data;
wire    [3:0]               mwr_dw_vld;
wire    [63:0]              mwr_addr;

wire                        cpld_wr_start;
wire    [9:0]               cpld_length;
wire    [6:0]               cpld_low_addr;
wire    [11:0]              cpld_byte_cnt;
wire    [127:0]             cpld_data;
wire    [3:0]               cpld_dw_vld;
wire                        multicpld_flag;

wire    [1:0]               bar_hit;

//mwr wr
wire                        mwr_wr_en;
wire    [ADDR_WIDTH-1:0]    mwr_wr_addr;
wire    [127:0]             mwr_wr_data;
wire    [15:0]              mwr_wr_be;
wire    [1:0]               mwr_wr_bar_hit/*synthesis PAP_MARK_DEBUG="1"*/;
//cpld wr
wire                        cpld_wr_en;
wire    [ADDR_WIDTH-1:0]    cpld_wr_addr;
wire    [127:0]             cpld_wr_data;
wire    [15:0]              cpld_wr_be;
wire    [1:0]               cpld_wr_bar_hit;
//bar0 wr interface
reg                         bar0_wr_en/*synthesis PAP_MARK_DEBUG="1"*/;
reg     [ADDR_WIDTH-1:0]    bar0_wr_addr;
reg     [127:0]             bar0_wr_data/*synthesis PAP_MARK_DEBUG="1"*/;
reg     [15:0]              bar0_wr_byte_en;
//bar2 wr interface
wire                        bar2_wr_en;
wire    [ADDR_WIDTH-1:0]    bar2_wr_addr;
wire    [127:0]             bar2_wr_data;
wire    [15:0]              bar2_wr_byte_en;

// Carry complete HDMI writes, including the source-frame epoch, into the
// PCIe clock domain.  The epoch travels with every FIFO entry; it is not a
// separately sampled pixclk pulse.
wire    [ADDR_WIDTH+129:0]  video_fifo_rd_data;
wire                        video_fifo_full;
wire                        video_fifo_empty;
wire                        video_fifo_rd_en;
wire                        video_ram_wr_en;
reg video_ram_commit, video_commit_end, video_commit_epoch;
reg [11:0] video_commit_addr;
reg [127:0] video_commit_data;
reg  [14:0]                 video_chunk_ready;
reg                         video_chunk_overflow;
reg  [1:0]                  video_slot_state [0:14];
reg                         video_chunk_accept;
reg                         video_epoch_valid;
reg                         video_epoch_seen;
reg                         video_wait_frame;
wire [11:0]                 video_fifo_addr;
wire                        video_fifo_epoch;
wire [3:0]                  video_fifo_slot;
wire                        video_fifo_first;
wire                        video_fifo_last;
wire                        video_fifo_frame_last;
wire                        video_fifo_chunk_end;
wire                        video_fifo_addr_valid;
wire                        video_epoch_transition;
wire                        video_consume_same_slot;
wire                        video_first_slot_free;
wire                        video_first_accept;
wire                        video_first_can_accept;
wire                        video_head_epoch_transition;
wire        [1:0]           video_fifo_slot_state;
integer                     video_slot_i;

localparam [11:0] VIDEO_STATUS_ADDR = 12'hfff;
localparam [1:0]  SLOT_FREE         = 2'd0;
localparam [1:0]  SLOT_FILLING      = 2'd1;
localparam [1:0]  SLOT_READY        = 2'd2;
localparam [1:0]  SLOT_DMA_IN_FLIGHT = 2'd3;

wire video_fifo_reset_n=rst_n && i_video_wr_rst_n;
reg [1:0] video_wr_reset_pipe, video_rd_reset_pipe;
always @(posedge i_video_wr_clk or negedge video_fifo_reset_n)
    if(!video_fifo_reset_n) video_wr_reset_pipe<=0;
    else video_wr_reset_pipe<={video_wr_reset_pipe[0],1'b1};
always @(posedge clk or negedge video_fifo_reset_n)
    if(!video_fifo_reset_n) video_rd_reset_pipe<=0;
    else video_rd_reset_pipe<={video_rd_reset_pipe[0],1'b1};
assign video_fifo_addr  = video_fifo_rd_data[ADDR_WIDTH+129:130];
assign video_fifo_epoch = video_fifo_rd_data[129];
assign video_fifo_frame_last = video_fifo_rd_data[128];
assign video_fifo_slot  = video_fifo_addr[11:8];
assign video_fifo_first = video_fifo_addr[7:0] == 8'h00;
assign video_fifo_last  = video_fifo_addr[7:0] == 8'hff;
assign video_fifo_chunk_end = video_fifo_last || video_fifo_frame_last;
assign video_fifo_addr_valid = (video_fifo_addr <= 12'heff) &&
                               (video_fifo_slot < 4'd15);
assign video_fifo_slot_state = (video_fifo_slot < 4'd15) ?
                               video_slot_state[video_fifo_slot] :
                               SLOT_DMA_IN_FLIGHT;
assign video_head_epoch_transition = !LOSSLESS_VIDEO_SOURCE && !video_fifo_empty &&
                                (!video_epoch_valid ||
                                 (video_fifo_epoch != video_epoch_seen));
assign video_epoch_transition = video_fifo_rd_en &&
                                video_head_epoch_transition;
assign video_consume_same_slot = i_video_chunk_consume &&
                                 (i_video_chunk_slot == video_fifo_slot) &&
                                 (i_video_chunk_slot < 4'd15);
assign video_first_slot_free = video_fifo_addr_valid &&
    ((video_fifo_slot_state == SLOT_FREE) ||
     ((video_fifo_slot_state == SLOT_DMA_IN_FLIGHT) &&
      video_consume_same_slot) ||
     (video_head_epoch_transition &&
      (video_fifo_slot_state != SLOT_DMA_IN_FLIGHT)));
assign video_first_can_accept = video_fifo_first &&
    !i_video_rearm && (!video_wait_frame || video_head_epoch_transition) &&
    video_first_slot_free;
assign video_fifo_rd_en = video_rd_reset_pipe[1] && ~video_fifo_empty &&
    (!LOSSLESS_VIDEO_SOURCE ||
     (video_fifo_first ? video_first_can_accept : video_chunk_accept));
assign video_first_accept = video_first_can_accept && video_fifo_rd_en;
assign video_ram_wr_en = video_fifo_rd_en && video_fifo_addr_valid &&
    (video_fifo_first ? video_first_accept :
     (!i_video_rearm && !video_wait_frame && video_chunk_accept));
assign o_video_chunk_ready = video_chunk_ready;
assign o_video_chunk_overflow = video_chunk_overflow;
assign o_video_frame_epoch = video_epoch_seen;
assign o_video_frame_error = video_chunk_overflow;
assign o_video_wr_ready = video_wr_reset_pipe[1] && ~video_fifo_full;

// Break the async FIFO address -> LUT RAM -> slot arbitration -> BRAM path.
// READY is published on the same edge that commits the final RAM word.
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        video_ram_commit <= 0; video_commit_addr <= 0; video_commit_data <= 0;
        video_commit_end <= 0; video_commit_epoch <= 0;
    end else if(i_video_rearm) begin
        video_ram_commit<=0;
    end else begin
        video_ram_commit <= video_ram_wr_en;
        if (video_ram_wr_en) begin
            video_commit_addr <= video_fifo_addr;
            video_commit_data <= video_fifo_rd_data[127:0];
            video_commit_end <= video_fifo_chunk_end;
            video_commit_epoch <= video_fifo_epoch;
        end
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        video_chunk_ready    <= 15'd0;
        video_chunk_overflow <= 1'b0;
        video_chunk_accept   <= 1'b0;
        video_epoch_valid    <= 1'b0;
        video_epoch_seen     <= 1'b0;
        video_wait_frame     <= 1'b0;
        for (video_slot_i = 0; video_slot_i < 15; video_slot_i = video_slot_i + 1)
            video_slot_state[video_slot_i] <= SLOT_FREE;
    end else begin
        if (i_video_rearm) begin
            // DDR frames may reuse the same physical frame-id. START is a
            // complete frame boundary there; an epoch edge is not required.
            video_wait_frame     <= LOSSLESS_VIDEO_SOURCE ? 1'b0 : 1'b1;
            video_chunk_accept   <= 1'b0;
            video_chunk_ready    <= 15'd0;
            video_chunk_overflow <= 1'b0;
            for (video_slot_i = 0; video_slot_i < 15; video_slot_i = video_slot_i + 1) begin
                if (video_slot_state[video_slot_i] != SLOT_DMA_IN_FLIGHT)
                    video_slot_state[video_slot_i] <= SLOT_FREE;
            end
        end else begin
            if (video_epoch_transition) begin
                video_epoch_valid <= 1'b1;
                video_epoch_seen  <= video_fifo_epoch;
                video_wait_frame  <= 1'b0;
                video_chunk_overflow <= 1'b0;
                // READY/FILLING belong to the old epoch. In-flight DMA is
                // retained so a new frame can never overwrite an active read.
                for (video_slot_i = 0; video_slot_i < 15; video_slot_i = video_slot_i + 1) begin
                    if (video_slot_state[video_slot_i] != SLOT_DMA_IN_FLIGHT) begin
                        video_slot_state[video_slot_i] <= SLOT_FREE;
                        video_chunk_ready[video_slot_i] <= 1'b0;
                    end
                end
            end

            if (i_video_chunk_consume && (i_video_chunk_slot < 4'd15)) begin
                if (video_slot_state[i_video_chunk_slot] == SLOT_DMA_IN_FLIGHT) begin
                    video_slot_state[i_video_chunk_slot] <= SLOT_FREE;
                    video_chunk_ready[i_video_chunk_slot] <= 1'b0;
                end else begin
                    video_chunk_overflow <= 1'b1;
                end
            end

            if (i_video_chunk_claim && (i_video_chunk_claim_slot < 4'd15)) begin
                if (video_slot_state[i_video_chunk_claim_slot] == SLOT_READY) begin
                    video_slot_state[i_video_chunk_claim_slot] <= SLOT_DMA_IN_FLIGHT;
                    video_chunk_ready[i_video_chunk_claim_slot] <= 1'b0;
                end else begin
                    video_chunk_overflow <= 1'b1;
                end
            end

            if (video_fifo_rd_en && video_fifo_first) begin
                video_chunk_accept <= video_first_accept;
                if (!video_first_accept)
                    video_chunk_overflow <= 1'b1;
                else
                    video_slot_state[video_fifo_slot] <= SLOT_FILLING;
            end

            if (video_ram_commit && video_commit_end &&
                (LOSSLESS_VIDEO_SOURCE ||
                 (video_commit_epoch == video_epoch_seen &&
                  !(video_fifo_rd_en && video_fifo_epoch != video_epoch_seen)))) begin
                video_slot_state[video_commit_addr[11:8]] <= SLOT_READY;
                video_chunk_ready[video_commit_addr[11:8]] <= 1'b1;
            end
            if (video_ram_wr_en && video_fifo_chunk_end)
                video_chunk_accept <= 1'b0;

            if (video_fifo_rd_en && !video_fifo_addr_valid)
                video_chunk_overflow <= 1'b1;
        end
    end
end

// These checks are deliberately simulation-only: they guard the address
// contract and the ownership invariant without changing the synthesis path.
// synthesis translate_off
initial begin
    if (VIDEO_STATUS_ADDR < 12'hf00)
        $fatal(1, "status address overlaps video slots: %03x", VIDEO_STATUS_ADDR);
end

always @(posedge clk) begin
    if (rst_n) begin
        if (video_ram_wr_en && (video_fifo_addr > 12'heff))
            $fatal(1, "video write entered reserved BAR0 window: %03x", video_fifo_addr);
        if (i_video_status_wr_en && (VIDEO_STATUS_ADDR < 12'hf00))
            $fatal(1, "status write is outside reserved BAR0 window: %03x", VIDEO_STATUS_ADDR);
        if (video_ram_wr_en && (video_fifo_slot < 4'd15) &&
            (video_slot_state[video_fifo_slot] == SLOT_DMA_IN_FLIGHT) &&
            !video_consume_same_slot)
            $fatal(1, "producer overwrote DMA_IN_FLIGHT slot %0d", video_fifo_slot);
    end
end
// synthesis translate_on

pgm_distributed_fifo_v1_1 #(
    .ADDR_WIDTH             (4),
    .DATA_WIDTH             (ADDR_WIDTH + 130),
    .RST_TYPE               ("ASYNC"),
    .OUT_REG                (0),
    .FIFO_TYPE              ("ASYNC_FIFO")
) u_video_write_fifo (
    .wr_data                ({i_video_wr_addr, i_video_frame_epoch,
                              i_video_wr_frame_last, i_video_wr_data}),
    .wr_en                  (i_video_wr_en && video_wr_reset_pipe[1]),
    .wr_clk                 (i_video_wr_clk),
    .full                   (video_fifo_full),
    .wr_rst                 (~video_wr_reset_pipe[1]),
    .almost_full            (),
    .wr_water_level         (),
    .rd_data                (video_fifo_rd_data),
    .rd_en                  (video_fifo_rd_en),
    .rd_clk                 (clk),
    .empty                  (video_fifo_empty),
    .rd_rst                 (~video_rd_reset_pipe[1]),
    .almost_empty           (),
    .rd_water_level         ()
);

ips2l_pcie_dma_tlp_rcv #(
    .DEVICE_TYPE            (DEVICE_TYPE            )
)
u_ips2l_pcie_dma_tlp_rcv
(
    .clk                    (clk                    ),  //gen1:62.5MHz,gen2:125MHz
    .rst_n                  (rst_n                  ),

    //**********************************************************************
    //AXIS master interface
    .i_axis_master_tvld     (i_axis_master_tvld     ),
    .o_axis_master_trdy     (o_axis_master_trdy     ),
    .i_axis_master_tdata    (i_axis_master_tdata    ),
    .i_axis_master_tkeep    (i_axis_master_tkeep    ),
    .i_axis_master_tlast    (i_axis_master_tlast    ),
    .i_axis_master_tuser    (i_axis_master_tuser    ),
    .o_trgt1_radm_pkt_halt  (o_trgt1_radm_pkt_halt  ),
//    .i_radm_grant_tlp_type  (i_radm_grant_tlp_type  ),

    //**********************************************************************
    //to mwr write control
    .o_mwr_wr_start         (mwr_wr_start           ),
    .o_mwr_length           (mwr_length             ),
    .o_mwr_dwbe             (mwr_dwbe               ),
    .o_mwr_data             (mwr_data               ),
    .o_mwr_dw_vld           (mwr_dw_vld             ),
    .o_mwr_addr             (mwr_addr               ),
    //to cpld write control
    .o_cpld_wr_start        (cpld_wr_start          ),
    .o_cpld_length          (cpld_length            ),
    .o_cpld_low_addr        (cpld_low_addr          ),
    .o_cpld_byte_cnt        (cpld_byte_cnt          ),
    .o_cpld_data            (cpld_data              ),
    .o_cpld_dw_vld          (cpld_dw_vld            ),
    .o_multicpld_flag       (multicpld_flag         ),
    //write bar hit
    .o_bar_hit              (bar_hit                ),
    //**********************************************************************
    //to tx top
    //req rcv
    .o_mrd_tc               (o_mrd_tc               ),
    .o_mrd_attr             (o_mrd_attr             ),
    .o_mrd_length           (o_mrd_length           ),
    .o_mrd_id               (o_mrd_id               ),
    .o_mrd_tag              (o_mrd_tag              ),
    .o_mrd_addr             (o_mrd_addr             ),

    .o_cpld_req_vld         (o_cpld_req_vld         ),
    .i_cpld_req_rdy         (i_cpld_req_rdy         ),
    .i_cpld_tx_rdy          (i_cpld_tx_rdy          ),
    //cpld rcv
    .o_cpld_rcv             (o_cpld_rcv             ),
    .o_cpld_tag             (o_cpld_tag             ),
    .i_tag_full             (i_tag_full             ),
    //rst tlp cnt
    .o_dma_check_result     (o_dma_check_result     ),
    .i_tx_restart           (i_tx_restart           )
    //.o_dbg_bus              (o_dbg_bus_rx_ctrl      ),
    //.o_dbg_tlp_rcv_cnt      (o_dbg_tlp_rcv_cnt      )
);


ips2l_pcie_dma_rx_cpld_wr_ctrl #(
    .ADDR_WIDTH             (ADDR_WIDTH             )
)
u_cpld_wr_ctrl
(
    .clk                    (clk                    ),  //gen1:62.5MHz,gen2:125MHz
    .rst_n                  (rst_n                  ),
    .i_cfg_max_rd_req_size  (i_cfg_max_rd_req_size  ),  //input [2:0]
    //****************************************
    .i_cpld_wr_start        (cpld_wr_start          ),
    .i_cpld_length          (cpld_length            ),
    .i_cpld_low_addr        (cpld_low_addr          ),
    .i_cpld_byte_cnt        (cpld_byte_cnt          ),
    .i_cpld_data            (cpld_data              ),
    .i_cpld_dw_vld          (cpld_dw_vld            ),
    .i_cpld_tag             (o_cpld_tag             ),
    .i_bar_hit              (bar_hit                ),
    .i_multicpld_flag       (multicpld_flag         ),
    //****************************************
    .o_cpld_wr_en           (cpld_wr_en             ),
    .o_cpld_wr_addr         (cpld_wr_addr           ),
    .o_cpld_wr_data         (cpld_wr_data           ),
    .o_cpld_wr_be           (cpld_wr_be             ),
    .o_cpld_wr_bar_hit      (cpld_wr_bar_hit        )
);

ips2l_pcie_dma_rx_mwr_wr_ctrl #(
    .ADDR_WIDTH             (ADDR_WIDTH             )
)
u_mwr_wr_ctrl
(
    .clk                    (clk                    ),  //gen1:62.5MHz,gen2:125MHz
    .rst_n                  (rst_n                  ),
    //****************************************
    .i_mwr_wr_start         (mwr_wr_start           ),
    .i_mwr_length           (mwr_length             ),
    .i_mwr_dwbe             (mwr_dwbe               ),
    .i_mwr_data             (mwr_data               ),
    .i_mwr_dw_vld           (mwr_dw_vld             ),
    .i_mwr_addr             (mwr_addr               ),
    .i_bar_hit              (bar_hit                ),
    //****************************************
    .o_mwr_wr_en            (mwr_wr_en              ),
    .o_mwr_wr_addr          (mwr_wr_addr            ),
    .o_mwr_wr_data          (mwr_wr_data            ),
    .o_mwr_wr_be            (mwr_wr_be              ),
    .o_mwr_wr_bar_hit       (mwr_wr_bar_hit         )
);

//bar0 interface
always@(*)
begin
    if(mwr_wr_bar_hit == 2'b0)
    begin
        bar0_wr_en       = mwr_wr_en;
        bar0_wr_addr     = mwr_wr_addr;
        bar0_wr_data     = mwr_wr_data;
        bar0_wr_byte_en  = mwr_wr_be;
    end
    else
    begin
        bar0_wr_en       = 1'b0;
        bar0_wr_addr     = {ADDR_WIDTH{1'b0}};
        bar0_wr_data     = 128'b0;
        bar0_wr_byte_en  = 16'b0;
    end
end
     
//bar1 interface 
always@(*)
begin
    if(bar_hit == 2'b1 && (DEVICE_TYPE == 3'b000 || DEVICE_TYPE == 3'b001))
    begin
        o_bar1_wr_en       = 1'b1;
        o_bar1_wr_addr     = mwr_addr[ADDR_WIDTH-1:0];
        o_bar1_wr_data     = mwr_data;
        o_bar1_wr_byte_en  = {{4{mwr_dwbe[3]}},{4{mwr_dwbe[2]}},{4{mwr_dwbe[1]}},{4{mwr_dwbe[0]}}};
    end
    else
    begin
        o_bar1_wr_en       = 1'b0;
        o_bar1_wr_addr     = {ADDR_WIDTH{1'b0}};
        o_bar1_wr_data     = 128'b0;
        o_bar1_wr_byte_en  = 16'b0;
    end
end

assign bar2_wr_en       = cpld_wr_en;
assign bar2_wr_addr     = cpld_wr_addr;
assign bar2_wr_data     = cpld_wr_data;
assign bar2_wr_byte_en  = cpld_wr_be;

// BAR words ff2..ff6 hold RXC1/LUX1/TIM1/CLK1/GEO1. Preserve their addresses at
// the RAM sink; redirecting them to fff corrupts the live VID1 page.
wire rx_diag_address = (i_video_status_wr_addr >= (LANE_STATUS ? 12'hfeb : (RX_CONTROL ? 12'hfef : 12'hff1))) &&
                       (i_video_status_wr_addr <= 12'hffe);
wire [11:0] selected_status_addr = (rx_diag_address === 1'b1) ?
    i_video_status_wr_addr : VIDEO_STATUS_ADDR;
// Video FIFO consumption has priority; retry diagnostics instead of losing
// one accepted video word when a status update coincides with a RAM write.
assign o_video_status_wr_ready = !video_ram_commit;
wire status_commit = i_video_status_wr_en && o_video_status_wr_ready;
ips2l_pcie_dma_ram ips2l_pcie_dma_bar0 (
    .wr_data            (status_commit ? i_video_status_wr_data : video_commit_data),
    .wr_addr            (status_commit ? selected_status_addr : video_commit_addr),
    .wr_en              (status_commit || video_ram_commit),
    .wr_byte_en         (16'hffff                   ),
    .wr_clk             (clk                        ),
    .wr_rst             (~rst_n                     ),
    .rd_addr            (i_bar0_rd_addr             ),  // input [8:0]
    .rd_data            (o_bar0_rd_data             ),  // output [127:0]
    .rd_clk             (clk                        ),  // input
    .rd_clk_en          (i_bar0_rd_clk_en           ),  // input
    .rd_rst             (~rst_n                     )   // input
);

ips2l_pcie_dma_ram ips2l_pcie_dma_bar2 (
    .wr_data            (bar2_wr_data               ),  // input [127:0]
    .wr_addr            (bar2_wr_addr               ),  // input [8:0]
    .wr_en              (bar2_wr_en                 ),  // input
    .wr_byte_en         (bar2_wr_byte_en            ),  // input [15:0]
    .wr_clk             (clk                        ),  // input
    .wr_rst             (~rst_n                     ),  // input
    .rd_addr            (i_bar2_rd_addr             ),  // input [8:0]
    .rd_data            (o_bar2_rd_data             ),  // output [127:0]
    .rd_clk             (clk                        ),  // input
    .rd_clk_en          (i_bar2_rd_clk_en           ),  // input
    .rd_rst             (~rst_n                     )   // input
);

ips2l_pcie_dma_ram ips2l_pcie_dma_video (
    .wr_data            (video_commit_data          ),
    .wr_addr            (video_commit_addr),
    .wr_en              (video_ram_commit           ),
    .wr_byte_en         (16'hffff                   ),
    .wr_clk             (clk                        ),
    .wr_rst             (~rst_n                     ),
    .rd_addr            (i_bar2_rd_addr             ),
    .rd_data            (o_video_dma_rd_data        ),
    .rd_clk             (clk                        ),
    .rd_clk_en          (i_bar2_rd_clk_en           ),
    .rd_rst             (~rst_n                     )
);

endmodule
