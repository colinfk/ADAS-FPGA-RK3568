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
// Filename:ips2l_pcie_dma.v
//////////////////////////////////////////////////////////////////////////////
module ips2l_pcie_dma #(
    parameter                           DEVICE_TYPE = 3'd0      ,   //3'd0:EP,3'd1:Legacy EP,3'd4:RC
    parameter   integer                 AXIS_SLAVE_NUM = 3      ,
    parameter                           ADDR_WIDTH  = 4'd12     ,
    parameter                           MWR_SOURCE_VIDEO = 1,
    parameter                           FRAME_BUFFER_SOURCE = 0,
    parameter integer                   FEATURE_TRAILER_WORDS = 0,
    parameter LANE_STATUS = 0,
    parameter RX_DIAGNOSTICS = 0,
    parameter RX_CONTROL = 0

)(
    input                               clk                     ,            //gen1:62.5MHz,gen2:125MHz
    input                               rst_n                   ,

    input                               i_video_wr_clk          ,
    input                               i_video_wr_rst_n        ,
    input                               i_video_wr_en           ,
    input           [ADDR_WIDTH-1:0]    i_video_wr_addr         ,
    input           [127:0]             i_video_wr_data         ,
    input                               i_video_frame_epoch     ,
    input                               i_video_wr_frame_last   ,
    output  wire                        o_video_wr_ready         ,
    output  wire                        o_video_source_frame_request,
    output  wire                        o_video_source_run,
    input                               i_video_source_idle,
    input                               i_external_status_wr_en ,
    input           [127:0]             i_external_status_wr_data,
    input i_lane_status_valid,output o_lane_status_ready,
    input [11:0] i_lane_status_addr,input [127:0] i_lane_status_data,
    input           [31:0]              i_frame_buffer_status  ,
    input           [31:0]              i_frame_buffer_seq     ,
    input           [127:0]             i_rx_diag, i_rx_geometry_diag,
    input [127:0] i_writer_diag, i_writer_fault_diag, i_capture_diag, i_queue_diag,
    input [127:0] i_geometry_fault_diag, i_clock_diag, i_timing_diag,
    input [127:0] i_low_light_diag,
    input [127:0] i_rx_config_diag,
    input [127:0] i_feature_diag,
    input [127:0] i_rx_control_diag, i_rx_pad_diag,
    output reg [31:0] o_rx_control_request,

    input           [7:0]               i_cfg_pbus_num          ,
    input           [4:0]               i_cfg_pbus_dev_num      ,
    input           [2:0]               i_cfg_max_rd_req_size   ,
    input           [2:0]               i_cfg_max_payload_size  ,
    //**********************************************************************
    //axis master interface
    input                               i_axis_master_tvld      ,
    output  wire                        o_axis_master_trdy      ,
    input           [127:0]             i_axis_master_tdata     ,
    input           [3:0]               i_axis_master_tkeep     ,
    input                               i_axis_master_tlast     ,
    input           [7:0]               i_axis_master_tuser     ,
    output  wire    [2:0]               o_trgt1_radm_pkt_halt   ,
//    input           [5:0]               i_radm_grant_tlp_type   ,
    //**********************************************************************
    //axis_slave0 interface
    input                               i_axis_slave0_trdy      ,
    output  wire                        o_axis_slave0_tvld      ,
    output  wire    [127:0]             o_axis_slave0_tdata     ,
    output  wire                        o_axis_slave0_tlast     ,
    output  wire                        o_axis_slave0_tuser     ,
    //axis_slave1 interface
    input                               i_axis_slave1_trdy      ,
    output  wire                        o_axis_slave1_tvld      ,
    output  wire    [127:0]             o_axis_slave1_tdata     ,
    output  wire                        o_axis_slave1_tlast     ,
    output  wire                        o_axis_slave1_tuser     ,
    //axis_slave2 interface
    input                               i_axis_slave2_trdy      ,
    output  wire                        o_axis_slave2_tvld      ,
    output  wire    [127:0]             o_axis_slave2_tdata     ,
    output  wire                        o_axis_slave2_tlast     ,
    output  wire                        o_axis_slave2_tuser     ,
    //**********************************************************************
    //credit interface
    //from pcie
    input                               i_cfg_ido_req_en        ,
    input                               i_cfg_ido_cpl_en        ,
    input           [7:0]               i_xadm_ph_cdts          ,
    input           [11:0]              i_xadm_pd_cdts          ,
    input           [7:0]               i_xadm_nph_cdts         ,
    input           [11:0]              i_xadm_npd_cdts         ,
    input           [7:0]               i_xadm_cplh_cdts        ,
    input           [11:0]              i_xadm_cpld_cdts        ,
    //**********************************************************************
    //apb interface
    input                               i_apb_psel              ,
    input           [8:0]               i_apb_paddr             ,
    input           [31:0]              i_apb_pwdata            ,
    input           [3:0]               i_apb_pstrb             ,
    input                               i_apb_pwrite            ,
    input                               i_apb_penable           ,
    output  wire                        o_apb_prdy              ,
    output  wire    [31:0]              o_apb_prdata            ,
    //cross_4kb_boundary
    output  wire                        o_cross_4kb_boundary
    //debug
    //output  wire    [159:0]             o_dbg_bus
);

//**********************************************************************
//from rx to tx
wire        [2:0]               mrd_tc;
wire        [2:0]               mrd_attr;
wire        [9:0]               mrd_length;
wire        [15:0]              mrd_id;
wire        [7:0]               mrd_tag;
wire        [63:0]              mrd_addr;

wire                            cpld_req_vld;
wire                            cpld_req_rdy;
wire                            cpld_tx_rdy;
//cpld rcv
wire                            cpld_rcv;
wire        [7:0]               cpld_tag;
wire                            tag_full;
//from dma_controller to tx
wire                            user_define_header_flag;
wire                            user_define_data_flag;
//mwr req & ack
wire                            mwr32_req;
wire                            mwr32_req_ack;
wire                            mwr64_req;
wire                            mwr64_req_ack;
//mrd req & ack
wire                            mrd32_req;
wire                            mrd32_req_ack;
wire                            mrd64_req;
wire                            mrd64_req_ack;
//req information
wire        [9:0]               req_length;
wire        [63:0]              req_addr;
wire        [31:0]              req_data;


//axis_slave0 interface
wire                            dma_axis_slave0_trdy ;
wire                            dma_axis_slave0_tvld ;
wire        [127:0]             dma_axis_slave0_tdata;
wire                            dma_axis_slave0_tlast;
wire                            dma_axis_slave0_tuser;
//axis_slave1 interface
wire                            dma_axis_slave1_trdy ;
wire                            dma_axis_slave1_tvld ;
wire        [127:0]             dma_axis_slave1_tdata;
wire                            dma_axis_slave1_tlast;
wire                            dma_axis_slave1_tuser;
//axis_slave2 interface
wire                            dma_axis_slave2_trdy ;
wire                            dma_axis_slave2_tvld ;
wire        [127:0]             dma_axis_slave2_tdata;
wire                            dma_axis_slave2_tlast;
wire                            dma_axis_slave2_tuser;

//**********************************************************************
//bar0 rd interface
wire                            bar0_rd_clk_en/*synthesis PAP_MARK_DEBUG="1"*/;
wire        [ADDR_WIDTH-1:0]    bar0_rd_addr;
wire        [127:0]             bar0_rd_data/*synthesis PAP_MARK_DEBUG="1"*/;
//bar1 wr interface
wire                            bar1_wr_en;
wire        [ADDR_WIDTH-1:0]    bar1_wr_addr;
wire        [127:0]             bar1_wr_data;
wire        [15:0]              bar1_wr_byte_en;
wire video_active, video_stopping;
// Reserved RX command slot, accepted only while the video DMA is idle.
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) o_rx_control_request<=0;
    else if (RX_CONTROL && bar1_wr_en && &bar1_wr_byte_en &&
             bar1_wr_addr==12'h0e0 && bar1_wr_data[31:24]==8'h72 &&
             !video_active && !video_stopping && i_video_source_idle===1'b1)
        o_rx_control_request<=bar1_wr_data[31:0];
end
//bar2 rd interface
wire                            bar2_rd_clk_en;
wire        [ADDR_WIDTH-1:0]    bar2_rd_addr;
wire        [127:0]             bar2_rd_data;
wire        [127:0]             video_dma_rd_data;
    wire        [14:0]              video_chunk_ready;
wire                            video_frame_epoch;
wire                            video_frame_error;
wire                            video_chunk_claim;
wire        [3:0]               video_chunk_claim_slot;
wire                            video_rearm;
    wire                            video_chunk_consume;
wire        [3:0]               video_chunk_consume_slot;
wire                            video_chunk_overflow;
wire                            video_frame_done;
wire        [31:0]              video_frame_seq;
wire                            video_buffer_select;
    wire                            video_overflow;
wire        [31:0]              video_dropped_frame_count;
wire                            video_auto_mwr64_req;
wire        [9:0]               video_auto_req_length;
wire        [63:0]              video_auto_req_addr;
wire        [31:0]              video_auto_req_data;
wire        [11:0]              video_source_base;
wire                            mwr_tx_busy;
wire                            video_status_wr_en;
wire        [127:0]             video_status_wr_data;
wire                            status_wr_en;
wire        [127:0]             status_wr_data;
wire [11:0] status_wr_addr;
wire status_wr_ready;
wire status_sink_ready;
reg status_write_valid;
reg [11:0] status_write_addr;
reg [127:0] status_write_data;
// One-entry elastic register isolates status assembly/dirty detection from
// BAR RAM muxes. Source arbitration advances only when this register accepts.
assign status_wr_ready=!status_write_valid || status_sink_ready;
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        status_write_valid<=0; status_write_addr<=12'hfff; status_write_data<=0;
    end else if(status_wr_ready) begin
        status_write_valid<=status_wr_en;
        if(status_wr_en) begin status_write_addr<=status_wr_addr; status_write_data<=status_wr_data; end
    end
end
wire [31:0] video_command_seq;
wire [127:0] dma_control_status={24'h050000,4'd0,video_overflow,
    (i_video_source_idle === 1'b1),video_stopping,video_active,
    video_command_seq,video_frame_seq,32'h444d4135};
reg [127:0] dma_control_previous;
reg dma_control_initialized, dma_control_dirty;
wire dma_control_wr_en=!dma_control_initialized || dma_control_dirty;
wire lane_status_request=LANE_STATUS && i_lane_status_valid;
wire lane_status_write=lane_status_request && !i_external_status_wr_en && !video_status_wr_en;
assign o_lane_status_ready=status_wr_ready && !i_external_status_wr_en && !video_status_wr_en;
wire dma_control_write=dma_control_wr_en && !i_external_status_wr_en && !video_status_wr_en && !lane_status_request;
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin dma_control_previous<=0;dma_control_initialized<=0;dma_control_dirty<=0;end
    else begin
        dma_control_dirty<=dma_control_status!=dma_control_previous;
        if(dma_control_write && status_wr_ready) begin
            dma_control_previous<=dma_control_status;dma_control_initialized<=1;
        end
    end
end
reg [15:0] rx_diag_timer;
reg rx_diag_pending;
reg [3:0] rx_diag_page;
reg [11:0] diag_address;
reg [127:0] diag_data;
always @(*) begin
    case(rx_diag_page)
        0: begin diag_address=12'hffe; diag_data=i_rx_diag; end
        1: begin diag_address=12'hffd; diag_data=i_rx_geometry_diag; end
        2: begin diag_address=12'hffc; diag_data=i_writer_diag; end
        3: begin diag_address=12'hffb; diag_data=i_writer_fault_diag; end
        4: begin diag_address=12'hffa; diag_data=i_capture_diag; end
        5: begin diag_address=12'hff9; diag_data={32'h07800438,32'd4147200,32'h20261007,32'h424c4431}; end
        6: begin diag_address=12'hff8; diag_data=i_queue_diag; end
        7: begin diag_address=12'hff6; diag_data=i_geometry_fault_diag; end
        8: begin diag_address=12'hff5; diag_data=i_clock_diag; end
        9: begin diag_address=12'hff4; diag_data=i_timing_diag; end
        10: begin diag_address=12'hff3; diag_data=i_low_light_diag; end
        11: begin diag_address=12'hff2; diag_data=i_rx_config_diag; end
        12: begin diag_address=12'hff1; diag_data=i_feature_diag; end
        13: begin diag_address=12'hff0; diag_data=i_rx_control_diag; end
        default: begin diag_address=12'hfef; diag_data=i_rx_pad_diag; end
    endcase
end
wire rx_diag_write = RX_DIAGNOSTICS && rx_diag_pending && !video_status_wr_en &&
                     !i_external_status_wr_en && !dma_control_wr_en && !lane_status_request;
wire                            mwr32_req_mux;
wire                            mwr64_req_mux;
wire                            video_mwr64_req_ack;
wire                            video_completion_data = FRAME_BUFFER_SOURCE &&
                                                        (video_auto_req_length == 10'd1);
wire                            legacy_mwr64_req_ack;
wire        [9:0]               req_length_mux;
wire        [63:0]              req_addr_mux;
wire        [31:0]              req_data_mux;

video_mwr64_arbiter u_video_mwr64_arbiter (
    .clk            (clk),
    .rst_n          (rst_n),
    .i_video_req    (video_auto_mwr64_req),
    .i_legacy_req   (mwr64_req),
    .i_tx_ack       (mwr64_req_ack),
    .i_tx_busy      (mwr_tx_busy),
    .o_tx_req       (mwr64_req_mux),
    .o_video_ack    (video_mwr64_req_ack),
    .o_legacy_ack   (legacy_mwr64_req_ack),
    .i_video_length (video_auto_req_length),
    .i_video_addr   (video_auto_req_addr),
    .i_video_data   (video_auto_req_data),
    .i_legacy_length(req_length),
    .i_legacy_addr  (req_addr),
    .i_legacy_data  (req_data),
    .o_req_length   (req_length_mux),
    .o_req_addr     (req_addr_mux),
    .o_req_data     (req_data_mux)
);
//**********************************************************************
//rst tlp cnt
wire                            tx_restart;
wire        [63:0]              dma_check_result;
//debug bus
//wire        [42:0]              dbg_bus_rx_ctrl;
//wire        [43:0]              dbg_bus_mrd_tx;
//wire        [72:0]              dbg_bus_mwr_tx;

//assign o_dbg_bus = {
//                    dbg_bus_mwr_tx, //159:87
//                    dbg_bus_mrd_tx, //86:43
//                    dbg_bus_rx_ctrl //42:0
//                    };

ips2l_pcie_dma_rx_top #(
    .DEVICE_TYPE                (DEVICE_TYPE                ),  //3'd0:EP,3'd1:Legacy EP,3'd4:RC
    .ADDR_WIDTH                 (ADDR_WIDTH                 ),
    .LANE_STATUS(LANE_STATUS),
    .LOSSLESS_VIDEO_SOURCE      (FRAME_BUFFER_SOURCE        ),
    .RX_CONTROL(RX_CONTROL)
)
u_ips2l_pcie_dma_rx_top
(
    .clk                        (clk                        ),  //gen1:62.5MHz,gen2:125MHz
    .rst_n                      (rst_n                      ),
    .i_video_wr_clk             (i_video_wr_clk             ),
    .i_video_wr_rst_n           (i_video_wr_rst_n           ),
    .i_video_wr_en              (i_video_wr_en              ),
    .i_video_wr_addr            (i_video_wr_addr            ),
    .i_video_wr_data            (i_video_wr_data            ),
    .i_video_frame_epoch        (i_video_frame_epoch        ),
    .i_video_wr_frame_last      (i_video_wr_frame_last      ),
    .o_video_wr_ready            (o_video_wr_ready            ),
    .i_cfg_max_rd_req_size      (i_cfg_max_rd_req_size      ),  //input [2:0]
    //**********************************************************************
    //axis master interface
    .i_axis_master_tvld         (i_axis_master_tvld         ),
    .o_axis_master_trdy         (o_axis_master_trdy         ),
    .i_axis_master_tdata        (i_axis_master_tdata        ),
    .i_axis_master_tkeep        (i_axis_master_tkeep        ),
    .i_axis_master_tlast        (i_axis_master_tlast        ),
    .i_axis_master_tuser        (i_axis_master_tuser        ),
    .o_trgt1_radm_pkt_halt      (o_trgt1_radm_pkt_halt      ),
//    .i_radm_grant_tlp_type      (i_radm_grant_tlp_type      ),
    //**********************************************************************
    //bar0 rd interface
    .i_bar0_rd_clk_en           (bar0_rd_clk_en             ),
    .i_bar0_rd_addr             (bar0_rd_addr               ),
    .o_bar0_rd_data             (bar0_rd_data               ),
    //bar2 rd interface
    .i_bar2_rd_clk_en           (bar2_rd_clk_en             ),
    .i_bar2_rd_addr             (bar2_rd_addr               ),
    .o_bar2_rd_data             (bar2_rd_data               ),
    .o_video_dma_rd_data        (video_dma_rd_data          ),
    .o_video_chunk_ready        (video_chunk_ready          ),
    .o_video_frame_epoch        (video_frame_epoch          ),
    .o_video_frame_error        (video_frame_error          ),
    .i_video_chunk_claim        (video_chunk_claim          ),
    .i_video_chunk_claim_slot   (video_chunk_claim_slot     ),
    .i_video_rearm              (video_rearm                ),
    .i_video_chunk_consume      (video_chunk_consume        ),
    .i_video_chunk_slot         (video_chunk_consume_slot   ),
    .o_video_chunk_overflow     (video_chunk_overflow       ),
    .i_video_status_wr_en       (status_write_valid         ),
    .i_video_status_wr_data     (status_write_data          ),
    .i_video_status_wr_addr(status_write_addr), .o_video_status_wr_ready(status_sink_ready),
    //bar1 wr interface
    .o_bar1_wr_en               (bar1_wr_en                 ),
    .o_bar1_wr_addr             (bar1_wr_addr               ),
    .o_bar1_wr_data             (bar1_wr_data               ),
    .o_bar1_wr_byte_en          (bar1_wr_byte_en            ),
    //**********************************************************************
    //to tx top
    //req rcv
    .o_mrd_tc                   (mrd_tc                     ),
    .o_mrd_attr                 (mrd_attr                   ),
    .o_mrd_length               (mrd_length                 ),
    .o_mrd_id                   (mrd_id                     ),
    .o_mrd_tag                  (mrd_tag                    ),
    .o_mrd_addr                 (mrd_addr                   ),

    .o_cpld_req_vld             (cpld_req_vld               ),
    .i_cpld_req_rdy             (cpld_req_rdy               ),
    .i_cpld_tx_rdy              (cpld_tx_rdy                ),
    //cpld rcv
    .o_cpld_rcv                 (cpld_rcv                   ),
    .o_cpld_tag                 (cpld_tag                   ),
    .i_tag_full                 (tag_full                   ),
    //rst tlp cnt
    .o_dma_check_result         (dma_check_result           ),
    .i_tx_restart               (tx_restart                 )

    //.o_dbg_bus_rx_ctrl          (dbg_bus_rx_ctrl            )
);

reg video_status_initialized;
reg video_status_dirty;
reg [127:0] video_status_previous;
video_dma_scheduler #(
    .FRAME_CHUNKS              (FRAME_BUFFER_SOURCE ? 1013 : 2025),
    .LAST_CHUNK_DWORDS         (FRAME_BUFFER_SOURCE ? 512+FEATURE_TRAILER_WORDS*4 : 1024),
    .FRAME_BUFFER_SOURCE        (FRAME_BUFFER_SOURCE        )
) u_video_dma_scheduler
(
    .clk                        (clk                        ),
    .rst_n                      (rst_n                      ),
    .i_cfg_wr_en                (bar1_wr_en                 ),
    .i_cfg_wr_addr              (bar1_wr_addr               ),
    .i_cfg_wr_data              (bar1_wr_data               ),
    .i_cfg_wr_byte_en           (bar1_wr_byte_en            ),
    .i_chunk_ready              (video_chunk_ready          ),
    .i_chunk_overflow           (video_frame_error          ),
    .i_frame_epoch              (video_frame_epoch          ),
    .i_source_idle              (i_video_source_idle        ),
    .i_mwr64_req_ack            (video_mwr64_req_ack        ),
    .i_mwr_tx_busy              (mwr_tx_busy                ),
    .o_mwr64_req                (video_auto_mwr64_req       ),
    .o_mwr32_req                (                             ),
    .o_req_length               (video_auto_req_length      ),
    .o_req_addr                 (video_auto_req_addr        ),
    .o_req_data                 (video_auto_req_data        ),
    .o_video_source_base        (video_source_base          ),
    .o_chunk_claim              (video_chunk_claim          ),
    .o_chunk_claim_slot         (video_chunk_claim_slot     ),
    .o_chunk_consume            (video_chunk_consume        ),
    .o_chunk_consume_slot       (video_chunk_consume_slot   ),
    .o_frame_done               (video_frame_done           ),
    .o_frame_seq                (video_frame_seq            ),
    .o_buffer_select            (video_buffer_select        ),
    .o_overflow                 (video_overflow             ),
    .o_video_rearm              (video_rearm                ),
    .o_source_frame_request     (o_video_source_frame_request),
    .o_dropped_frame_count      (video_dropped_frame_count  ),
    .o_source_run(o_video_source_run), .o_stopping(video_stopping),
    .o_command_seq(video_command_seq),
    .o_active                   (video_active               )
);

// Publish idle at reset and every status change, not only frame completion.
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        video_status_initialized <= 1'b0;
        video_status_previous <= 128'd0;
    end else if (video_status_wr_en && !i_external_status_wr_en && status_wr_ready) begin
        video_status_initialized <= 1'b1;
        video_status_previous <= video_status_wr_data;
    end
end
// A registered dirty indication removes a 128-bit compare from the write
// enable/data path to BAR RAM. A coincident new change is detected next cycle.
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) video_status_dirty<=1;
    else if(video_status_wr_en && !i_external_status_wr_en && status_wr_ready)
        video_status_dirty<=0;
    else if(video_status_previous != video_status_wr_data)
        video_status_dirty<=1;
end
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin rx_diag_timer <= 0; rx_diag_pending <= 0; rx_diag_page <= 0; end
    else begin
        rx_diag_timer <= rx_diag_timer + 1'b1;
        if (&rx_diag_timer) rx_diag_pending <= 1;
        if (rx_diag_write && status_wr_ready) begin
            rx_diag_pending <= 0;
            rx_diag_page <= (rx_diag_page==(RX_CONTROL ? 14 : (FEATURE_TRAILER_WORDS!=0 ? 12 : 11))) ? 0 : rx_diag_page+1'b1;
        end
    end
end
assign video_status_wr_en   = !video_status_initialized || video_status_dirty;
assign video_status_wr_data = FRAME_BUFFER_SOURCE ?
                              {i_frame_buffer_seq, i_frame_buffer_status,
                               video_frame_seq, 32'h56494431} :
                              {video_dropped_frame_count, 29'd0, video_active,
                               video_overflow, video_buffer_select,
                               video_frame_seq, 32'h56494430};
assign status_wr_en = lane_status_write || i_external_status_wr_en || video_status_wr_en || dma_control_write || rx_diag_write;
assign status_wr_addr = lane_status_write ? i_lane_status_addr : dma_control_write ? 12'hff7 : (rx_diag_write ? diag_address : 12'hfff);
assign status_wr_data       = i_external_status_wr_en ?
                              i_external_status_wr_data :
    (lane_status_write ? i_lane_status_data : dma_control_write ? dma_control_status : (rx_diag_write ? diag_data : video_status_wr_data));
assign mwr32_req_mux        = mwr32_req;

ips2l_pcie_dma_controller #(
    .DEVICE_TYPE                (DEVICE_TYPE                ),  //3'd0:EP,3'd1:Legacy EP,3'd4:RC
    .ADDR_WIDTH                 (ADDR_WIDTH                 )
)
u_ips2l_pcie_dma_controller
(
    .clk                        (clk                        ),  //gen1:62.5MHz,gen2:125MHz
    .rst_n                      (rst_n                      ),
    //**********************************************************************
    //bar1 wr interface
    .i_bar1_wr_en               (bar1_wr_en                 ),
    .i_bar1_wr_addr             (bar1_wr_addr               ),
    .i_bar1_wr_data             (bar1_wr_data               ),
    .i_bar1_wr_byte_en          (bar1_wr_byte_en            ),
    //**********************************************************************
    //apb interface
    .i_apb_psel                 (i_apb_psel                 ),
    .i_apb_paddr                (i_apb_paddr                ),
    .i_apb_pwdata               (i_apb_pwdata               ),
    .i_apb_pstrb                (i_apb_pstrb                ),
    .i_apb_pwrite               (i_apb_pwrite               ),
    .i_apb_penable              (i_apb_penable              ),
    .o_apb_prdy                 (o_apb_prdy                 ),
    .o_apb_prdata               (o_apb_prdata               ),
    //**********************************************************************
    .o_user_define_data_flag    (user_define_data_flag      ),

    //**********************************************************************
    //to tx top
    //mwr req & ack
    .o_mwr32_req                (mwr32_req                  ),
    .i_mwr32_req_ack            (mwr32_req_ack              ),
    .o_mwr64_req                (mwr64_req                  ),
    .i_mwr64_req_ack            (legacy_mwr64_req_ack       ),
    //mrd req & ack
    .o_mrd32_req                (mrd32_req                  ),
    .i_mrd32_req_ack            (mrd32_req_ack              ),
    .o_mrd64_req                (mrd64_req                  ),
    .i_mrd64_req_ack            (mrd64_req_ack              ),
    //req information
    .o_req_length               (req_length                 ),
    .o_req_addr                 (req_addr                   ),
    .o_req_data                 (req_data                   ),
    .o_cross_4kb_boundary       (o_cross_4kb_boundary       ),
    //rst tlp cnt
    .i_dma_check_result         (dma_check_result           ),
    .o_tx_restart               (tx_restart                 )
);

ips2l_pcie_dma_tx_top #(
//    .DEVICE_TYPE                (DEVICE_TYPE                ),          //3'd0:EP,3'd1:Legacy EP,3'd4:RC
    .ADDR_WIDTH                 (ADDR_WIDTH                 ),
    .MWR_SOURCE_VIDEO           (MWR_SOURCE_VIDEO           )
)
u_ips2l_pcie_dma_tx_top
(
    .clk                        (clk                        ),  //gen1:62.5MHz,gen2:125MHz
    .rst_n                      (rst_n                      ),
    .i_cfg_pbus_num             (i_cfg_pbus_num             ),  //input [7:0]
    .i_cfg_pbus_dev_num         (i_cfg_pbus_dev_num         ),  //input [4:0]
    .i_cfg_max_rd_req_size      (i_cfg_max_rd_req_size      ),  //input [2:0]
    .i_cfg_max_payload_size     (i_cfg_max_payload_size     ),  //input [2:0]
    //**********************************************************************
    //axis_slave0 interface
    .i_axis_slave0_trdy         (dma_axis_slave0_trdy       ),
    .o_axis_slave0_tvld         (dma_axis_slave0_tvld       ),
    .o_axis_slave0_tdata        (dma_axis_slave0_tdata      ),
    .o_axis_slave0_tlast        (dma_axis_slave0_tlast      ),
    .o_axis_slave0_tuser        (dma_axis_slave0_tuser      ),
    //axis_slave1 interface
    .i_axis_slave1_trdy         (dma_axis_slave1_trdy       ),
    .o_axis_slave1_tvld         (dma_axis_slave1_tvld       ),
    .o_axis_slave1_tdata        (dma_axis_slave1_tdata      ),
    .o_axis_slave1_tlast        (dma_axis_slave1_tlast      ),
    .o_axis_slave1_tuser        (dma_axis_slave1_tuser      ),
    //axis_slave2 interface
    .i_axis_slave2_trdy         (dma_axis_slave2_trdy       ),
    .o_axis_slave2_tvld         (dma_axis_slave2_tvld       ),
    .o_axis_slave2_tdata        (dma_axis_slave2_tdata      ),
    .o_axis_slave2_tlast        (dma_axis_slave2_tlast      ),
    .o_axis_slave2_tuser        (dma_axis_slave2_tuser      ),
    //**********************************************************************
    //from dma_controller
    .i_user_define_data_flag    (user_define_data_flag |
                                 video_completion_data      ),

    .i_mwr32_req                (mwr32_req_mux              ),
    .o_mwr32_req_ack            (mwr32_req_ack              ),
    .i_mwr64_req                (mwr64_req_mux              ),
    .o_mwr64_req_ack            (mwr64_req_ack              ),

    .i_mrd32_req                (mrd32_req                  ),
    .o_mrd32_req_ack            (mrd32_req_ack              ),
    .i_mrd64_req                (mrd64_req                  ),
    .o_mrd64_req_ack            (mrd64_req_ack              ),

    .i_req_length               (req_length_mux              ),
    .i_req_addr                 (req_addr_mux                ),
    .i_req_data                 (req_data_mux                ),

    //**********************************************************************
    //bar0 rd interface
    .o_bar0_rd_clk_en           (bar0_rd_clk_en             ),
    .o_bar0_rd_addr             (bar0_rd_addr               ),
    .i_bar0_rd_data             (bar0_rd_data               ),
    //bar2 rd interface
    .o_bar2_rd_clk_en           (bar2_rd_clk_en             ),
    .o_bar2_rd_addr             (bar2_rd_addr               ),
    .i_bar2_rd_data             (bar2_rd_data               ),
    .i_video_dma_rd_data        (video_dma_rd_data          ),
    .i_video_source_base        (video_source_base          ),
    //**********************************************************************
    //from rx top
    //req rcv
    .i_mrd_tc                   (mrd_tc                     ),
    .i_mrd_attr                 (mrd_attr                   ),
    .i_mrd_length               (mrd_length                 ),
    .i_mrd_id                   (mrd_id                     ),
    .i_mrd_tag                  (mrd_tag                    ),
    .i_mrd_addr                 (mrd_addr                   ),

    .i_cpld_req_vld             (cpld_req_vld               ),
    .o_cpld_req_rdy             (cpld_req_rdy               ),
    .o_cpld_tx_rdy              (cpld_tx_rdy                ),
    //cpld rcv
    .i_cpld_rcv                 (cpld_rcv                   ),
    .i_cpld_tag                 (cpld_tag                   ),
    .o_tag_full                 (tag_full                   ),
    //rst tlp cnt
    .o_mwr_tx_busy              (mwr_tx_busy                ),
    .i_tx_restart               (tx_restart                 )
    //debug
    //.o_dbg_bus_mrd_tx           (dbg_bus_mrd_tx             ),
    //.o_dbg_bus_mwr_tx           (dbg_bus_mwr_tx             )
);

generate
    if (AXIS_SLAVE_NUM == 1)
    begin:axis_slave_num_1
        ips2l_pcie_dma_tlp_tx_mux #(
            .AXIS_SLAVE_NUM             (AXIS_SLAVE_NUM         )
        )
        u_pcie_dma_tlp_tx_mux
        (
            .clk                         (clk                   ),   //gen1:62.5MHz,gen2:125MHz
            .rst_n                       (rst_n                 ),
            //**********************************************************************
            //from dma
            //axis_slave0 interface
            .o_dma_axis_slave0_trdy      (dma_axis_slave0_trdy  ),
            .i_dma_axis_slave0_tvld      (dma_axis_slave0_tvld  ),
            .i_dma_axis_slave0_tdata     (dma_axis_slave0_tdata ),
            .i_dma_axis_slave0_tlast     (dma_axis_slave0_tlast ),
            .i_dma_axis_slave0_tuser     (dma_axis_slave0_tuser ),
            //axis_slave1 interface
            .o_dma_axis_slave1_trdy      (dma_axis_slave1_trdy  ),
            .i_dma_axis_slave1_tvld      (dma_axis_slave1_tvld  ),
            .i_dma_axis_slave1_tdata     (dma_axis_slave1_tdata ),
            .i_dma_axis_slave1_tlast     (dma_axis_slave1_tlast ),
            .i_dma_axis_slave1_tuser     (dma_axis_slave1_tuser ),
            //axis_slave2 interface
            .o_dma_axis_slave2_trdy      (dma_axis_slave2_trdy  ),
            .i_dma_axis_slave2_tvld      (dma_axis_slave2_tvld  ),
            .i_dma_axis_slave2_tdata     (dma_axis_slave2_tdata ),
            .i_dma_axis_slave2_tlast     (dma_axis_slave2_tlast ),
            .i_dma_axis_slave2_tuser     (dma_axis_slave2_tuser ),
            //credit interface
            //from pcie
            .i_cfg_ido_req_en            (i_cfg_ido_req_en      ),
            .i_cfg_ido_cpl_en            (i_cfg_ido_cpl_en      ),
            .i_xadm_ph_cdts              (i_xadm_ph_cdts        ),
            .i_xadm_pd_cdts              (i_xadm_pd_cdts        ),
            .i_xadm_nph_cdts             (i_xadm_nph_cdts       ),
            .i_xadm_npd_cdts             (i_xadm_npd_cdts       ),
            .i_xadm_cplh_cdts            (i_xadm_cplh_cdts      ),
            .i_xadm_cpld_cdts            (i_xadm_cpld_cdts      ),
            //pcie_axis_slave0
            .i_pcie_axis_slave_trdy      (i_axis_slave0_trdy    ),
            .o_pcie_axis_slave_tvld      (o_axis_slave0_tvld    ),
            .o_pcie_axis_slave_tdata     (o_axis_slave0_tdata   ),
            .o_pcie_axis_slave_tlast     (o_axis_slave0_tlast   ),
            .o_pcie_axis_slave_tuser     (o_axis_slave0_tuser   )
        );

        assign o_axis_slave1_tvld  = 1'b0;
        assign o_axis_slave1_tdata = 128'b0;
        assign o_axis_slave1_tlast = 1'b0;
        assign o_axis_slave1_tuser = 1'b0;

        assign o_axis_slave2_tvld  = 1'b0;
        assign o_axis_slave2_tdata = 128'b0;
        assign o_axis_slave2_tlast = 1'b0;
        assign o_axis_slave2_tuser = 1'b0;
    end
    else if (AXIS_SLAVE_NUM == 2)
    begin:axis_slave_num_2
        ips2l_pcie_dma_tlp_tx_mux #(
            .AXIS_SLAVE_NUM             (AXIS_SLAVE_NUM         )
        )
        u_pcie_dma_tlp_tx_mux
        (
            .clk                         (clk                   ),   //gen1:62.5MHz,gen2:125MHz
            .rst_n                       (rst_n                 ),
            //**********************************************************************
            //from dma
            //axis_slave0 interface
            .o_dma_axis_slave0_trdy      (                      ),
            .i_dma_axis_slave0_tvld      (1'b0                  ),
            .i_dma_axis_slave0_tdata     (128'b0                ),
            .i_dma_axis_slave0_tlast     (1'b0                  ),
            .i_dma_axis_slave0_tuser     (1'b0                  ),
            //axis_slave1 interface
            .o_dma_axis_slave1_trdy      (dma_axis_slave1_trdy  ),
            .i_dma_axis_slave1_tvld      (dma_axis_slave1_tvld  ),
            .i_dma_axis_slave1_tdata     (dma_axis_slave1_tdata ),
            .i_dma_axis_slave1_tlast     (dma_axis_slave1_tlast ),
            .i_dma_axis_slave1_tuser     (dma_axis_slave1_tuser ),
            //axis_slave2 interface
            .o_dma_axis_slave2_trdy      (dma_axis_slave2_trdy  ),
            .i_dma_axis_slave2_tvld      (dma_axis_slave2_tvld  ),
            .i_dma_axis_slave2_tdata     (dma_axis_slave2_tdata ),
            .i_dma_axis_slave2_tlast     (dma_axis_slave2_tlast ),
            .i_dma_axis_slave2_tuser     (dma_axis_slave2_tuser ),
            //credit interface
            //from pcie
            .i_cfg_ido_req_en            (i_cfg_ido_req_en      ),
            .i_cfg_ido_cpl_en            (i_cfg_ido_cpl_en      ),
            .i_xadm_ph_cdts              (i_xadm_ph_cdts        ),
            .i_xadm_pd_cdts              (i_xadm_pd_cdts        ),
            .i_xadm_nph_cdts             (i_xadm_nph_cdts       ),
            .i_xadm_npd_cdts             (i_xadm_npd_cdts       ),
            .i_xadm_cplh_cdts            (i_xadm_cplh_cdts      ),
            .i_xadm_cpld_cdts            (i_xadm_cpld_cdts      ),
            //pcie_axis_slave0
            .i_pcie_axis_slave_trdy      (i_axis_slave1_trdy    ),
            .o_pcie_axis_slave_tvld      (o_axis_slave1_tvld    ),
            .o_pcie_axis_slave_tdata     (o_axis_slave1_tdata   ),
            .o_pcie_axis_slave_tlast     (o_axis_slave1_tlast   ),
            .o_pcie_axis_slave_tuser     (o_axis_slave1_tuser   )
        );

        assign dma_axis_slave0_trdy = i_axis_slave0_trdy    ;
        assign o_axis_slave0_tvld   = dma_axis_slave0_tvld  ;
        assign o_axis_slave0_tdata  = dma_axis_slave0_tdata ;
        assign o_axis_slave0_tlast  = dma_axis_slave0_tlast ;
        assign o_axis_slave0_tuser  = dma_axis_slave0_tuser ;

        assign o_axis_slave2_tvld  = 1'b0;
        assign o_axis_slave2_tdata = 128'b0;
        assign o_axis_slave2_tlast = 1'b0;
        assign o_axis_slave2_tuser = 1'b0;
    end
    else if (AXIS_SLAVE_NUM == 3)
    begin:axis_slave_num_3
        assign dma_axis_slave0_trdy = i_axis_slave0_trdy    ;
        assign o_axis_slave0_tvld   = dma_axis_slave0_tvld  ;
        assign o_axis_slave0_tdata  = dma_axis_slave0_tdata ;
        assign o_axis_slave0_tlast  = dma_axis_slave0_tlast ;
        assign o_axis_slave0_tuser  = dma_axis_slave0_tuser ;

        assign dma_axis_slave1_trdy = i_axis_slave1_trdy    ;
        assign o_axis_slave1_tvld   = dma_axis_slave1_tvld  ;
        assign o_axis_slave1_tdata  = dma_axis_slave1_tdata ;
        assign o_axis_slave1_tlast  = dma_axis_slave1_tlast ;
        assign o_axis_slave1_tuser  = dma_axis_slave1_tuser ;

        assign dma_axis_slave2_trdy = i_axis_slave2_trdy    ;
        assign o_axis_slave2_tvld   = dma_axis_slave2_tvld  ;
        assign o_axis_slave2_tdata  = dma_axis_slave2_tdata ;
        assign o_axis_slave2_tlast  = dma_axis_slave2_tlast ;
        assign o_axis_slave2_tuser  = dma_axis_slave2_tuser ;
    end
endgenerate

endmodule

module video_mwr64_arbiter (
    input               clk,
    input               rst_n,
    input               i_video_req,
    input               i_legacy_req,
    input               i_tx_ack,
    input               i_tx_busy,
    output wire         o_tx_req,
    output wire         o_video_ack,
    output wire         o_legacy_ack,
    input       [9:0]   i_video_length,
    input       [63:0]  i_video_addr,
    input       [31:0]  i_video_data,
    input       [9:0]   i_legacy_length,
    input       [63:0]  i_legacy_addr,
    input       [31:0]  i_legacy_data,
    output wire [9:0]   o_req_length,
    output wire [63:0]  o_req_addr,
    output wire [31:0]  o_req_data
);

localparam [1:0] GRANT_NONE   = 2'd0;
localparam [1:0] GRANT_VIDEO  = 2'd1;
localparam [1:0] GRANT_LEGACY = 2'd2;
reg [1:0] grant;
reg       busy_seen;

assign o_tx_req = (grant == GRANT_VIDEO)  ? i_video_req :
                  (grant == GRANT_LEGACY) ? i_legacy_req : 1'b0;
assign o_video_ack  = (grant == GRANT_VIDEO)  ? i_tx_ack : 1'b0;
assign o_legacy_ack = (grant == GRANT_LEGACY) ? i_tx_ack : 1'b0;
assign o_req_length = (grant == GRANT_VIDEO) ? i_video_length : i_legacy_length;
assign o_req_addr   = (grant == GRANT_VIDEO) ? i_video_addr : i_legacy_addr;
assign o_req_data   = (grant == GRANT_VIDEO) ? i_video_data : i_legacy_data;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        grant     <= GRANT_NONE;
        busy_seen <= 1'b0;
    end else begin
        case (grant)
            GRANT_NONE: begin
                busy_seen <= 1'b0;
                // Preserve legacy DMA when both request sources arrive together.
                if (i_legacy_req)
                    grant <= GRANT_LEGACY;
                else if (i_video_req)
                    grant <= GRANT_VIDEO;
            end
            default: begin
                if (i_tx_busy)
                    busy_seen <= 1'b1;
                if (busy_seen && !i_tx_busy) begin
                    grant     <= GRANT_NONE;
                    busy_seen <= 1'b0;
                end
            end
        endcase
    end
end

endmodule
