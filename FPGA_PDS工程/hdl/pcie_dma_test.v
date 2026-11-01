//pango参考例程
//基于官方demo修改
module pcie_dma_test #(
	parameter USE_DDR_FRAME_BUFFER = 1'b1,
	// 0 = independent DDR BIST bring-up, 1 = HDMI CDC/FIFO to DDR writer.
	parameter DDR_FRAME_BUFFER_MODE = 1'b1,
    // 0 raw, 1 regional automatic, 2 fixed strength, 3 left raw/right automatic.
    parameter [1:0] LOW_LIGHT_MODE = 2'd1,
    parameter integer LOW_LIGHT_PROFILE = 3,
    parameter [8:0] LOW_LIGHT_MANUAL_STRENGTH = 9'd256,
    parameter ENABLE_FPGA_FEATURE = 1'b1,
    parameter ENABLE_TINY_CNN = 1'b1,
    parameter INTERNAL_VIDEO_TEST = 1'b0
) (
	input					key_left_n,	// KEY0 / M15: left request
	input					sys_clk,

	// PG2L100H DDR3 reference design interface (16-bit, 512MB device)
	output					mem_cs_n,
	output					mem_rst_n,
	output					mem_ck,
	output					mem_ck_n,
	output					mem_cke,
	output					mem_ras_n,
	output					mem_cas_n,
	output					mem_we_n,
	output					mem_odt,
	output	[14:0]		mem_a,
	output	[2:0]		mem_ba,
	inout	[1:0]		mem_dqs,
	inout	[1:0]		mem_dqs_n,
	inout	[15:0]		mem_dq,
	output	[1:0]		mem_dm,

	// HDMIK V1.1 / MS7200 parallel video input
	output					hd_scl,
	inout					hd_sda,
	input					pixclk_in,
	input					vs_in,
	input					hs_in,
	input					de_in,
	input		[7:0]		r_in,
	input		[7:0]		g_in,
	input		[7:0]		b_in,

	// PCIE interface
	input					ref_clk_p,		// 输入参考时钟 
	input					ref_clk_n,		// 
	input					key_right_n,	// KEY1 / M16: right request
	input		[1:0]		rxn,			//pcie接收
	input		[1:0]		rxp,			//
	output wire	[1:0]		txn,			//pcie发送
	output wire	[1:0]		txp,			// 

	// LED signals
	output reg				ref_led,    
	output reg				pclk_led
);

// PLL lock is the asynchronous power-reset source. Native reset trees synchronize release; keys never reset them.
wire board_pll_locked;
wire board_power_rst_n=board_pll_locked;
localparam DEVICE_TYPE = 3'b000;			// @IPC enum 3'b000, 3'b001, 3'b100
localparam AXIS_SLAVE_NUM = 3;				// @IPC enum 1 2 3
localparam DDR_VIDEO_ACTIVE = USE_DDR_FRAME_BUFFER && DDR_FRAME_BUFFER_MODE;
localparam FEATURE_TRAILER_WORDS=ENABLE_FPGA_FEATURE ? 16 : 0;
wire capture_clk=INTERNAL_VIDEO_TEST ? sys_clk : pixclk_in;
wire capture_input_ready,capture_input_vs,capture_input_hs,capture_input_de;
wire [7:0] capture_input_r,capture_input_g,capture_input_b;
wire [31:0] capture_source_frame_id, capture_token_source_id;
wire feature_packet_valid;
wire [511:0] feature_packet;
wire [127:0] feature_append_diag;
wire [127:0] feature_append_diag_pcie;
wire feature_stream_valid,feature_stream_sof,feature_stream_eof,feature_stream_abort,feature_stream_epoch;
wire [127:0] feature_stream_data;
wire [19:0] feature_stream_index;

// Test unit mode signals
wire			pcie_cfg_ctrl_en;			
wire			axis_master_tready_cfg;		

wire			cfg_axis_slave0_tvalid;		
wire	[127:0]	cfg_axis_slave0_tdata;		
wire			cfg_axis_slave0_tlast;		
wire			cfg_axis_slave0_tuser;		

// For mux
wire			axis_master_tready_mem;		
wire			axis_master_tvalid_mem;		
wire	[127:0]	axis_master_tdata_mem;		
wire	[3:0]	axis_master_tkeep_mem;		
											
wire			axis_master_tlast_mem;		
wire	[7:0]	axis_master_tuser_mem;		

wire			cross_4kb_boundary;			

// Legacy 15-slot direct-stream path. Keep it for debug/benchmark; the final
// video path will source PCIe MWr data from the DDR frame buffer.
wire			video_wr_en;
wire	[11:0]	video_wr_addr;
wire	[127:0]	video_wr_data;
wire			video_capture_done;
wire			video_frame_epoch;
wire			video_frame_stream_valid;
wire [127:0]	video_frame_stream_data;
wire			video_frame_stream_epoch;
wire			video_frame_stream_sof;
wire			video_frame_stream_eof;
wire [19:0]		video_frame_stream_word_index;
wire			video_ddr_stream_valid;
wire [127:0]	video_ddr_stream_data;
wire			video_ddr_stream_epoch;
wire			video_ddr_stream_sof;
wire			video_ddr_stream_eof;
wire [19:0]		video_ddr_stream_word_index;
wire			hdmi_init_rst_n;
wire            hdmi_rx_init_done;
wire [7:0]      hdmi_rx_flags;
wire [127:0]    hdmi_rx_diag, hdmi_rx_geometry_diag;
wire [127:0] hdmi_config_diag, config_diag_pcie;
wire [127:0] hdmi_control_diag,control_diag_pcie,hdmi_pad_diag,pad_diag_pcie;
wire [31:0] hdmi_control_request,control_request_cfg;
wire [127:0] hdmi_clock_diag,hdmi_timing_diag,clock_diag_pcie,timing_diag_pcie;
wire [127:0] geometry_fault_pix,geometry_fault_pcie;
wire hdmi_mode_valid, video_vs, video_hs, video_de;
wire [7:0] video_r, video_g, video_b;
wire processed_ready, processed_vs, processed_de;
wire [7:0] processed_r, processed_g, processed_b;
wire [127:0] low_light_diag_pix, low_light_diag_pcie;
wire hdmi_cfg_clk, video_ddr_stream_abort;
wire [127:0] capture_diag_pix, capture_diag_pcie;
wire [127:0] writer_diag_ddr, writer_fault_diag_ddr, writer_diag_pcie, writer_fault_diag_pcie;
wire [127:0] queue_diag_ddr, queue_diag_pcie;
wire            ddr_ownership_drop, ddr_writer_fault;
reg [7:0]      hdmi_flags_d1, hdmi_flags_d2;
wire [127:0] hdmi_diag_d2, hdmi_geometry_d2;
cdc_snapshot #(.WIDTH(896)) u_hdmi_diag_snapshot (
    .src_clk(hdmi_cfg_clk),.src_rst_n(hdmi_init_rst_n),
    .src_data({hdmi_control_diag,hdmi_pad_diag,hdmi_rx_diag,hdmi_rx_geometry_diag,hdmi_clock_diag,hdmi_timing_diag,hdmi_config_diag}),
    .dst_clk(pclk_div2),.dst_rst_n(core_rst_n),
    .dst_data({control_diag_pcie,pad_diag_pcie,hdmi_diag_d2,hdmi_geometry_d2,clock_diag_pcie,timing_diag_pcie,config_diag_pcie}));
cdc_snapshot #(.WIDTH(32)) u_rx_request_snapshot (
    .src_clk(pclk_div2),.src_rst_n(core_rst_n),.src_data(hdmi_control_request),
    .dst_clk(hdmi_cfg_clk),.dst_rst_n(hdmi_init_rst_n),.dst_data(control_request_cfg));
cdc_snapshot #(.WIDTH(256)) u_capture_diag_snapshot (
    .src_clk(capture_clk),.src_rst_n(pix_capture_rst_n),.src_data({capture_diag_pix,geometry_fault_pix}),
    .dst_clk(pclk_div2),.dst_rst_n(core_rst_n),.dst_data({capture_diag_pcie,geometry_fault_pcie}));
reg [1:0]      ddr_extra_d1, ddr_extra_d2;
cdc_snapshot #(.WIDTH(128)) u_low_light_diag_snapshot (
    .src_clk(capture_clk),.src_rst_n(pix_capture_rst_n),.src_data(low_light_diag_pix),
    .dst_clk(pclk_div2),.dst_rst_n(core_rst_n),.dst_data(low_light_diag_pcie));
cdc_snapshot #(.WIDTH(128)) u_feature_append_diag_snapshot (
    .src_clk(capture_clk),.src_rst_n(pix_capture_rst_n),.src_data(feature_append_diag),
    .dst_clk(pclk_div2),.dst_rst_n(core_rst_n),.dst_data(feature_append_diag_pcie));
always @(posedge pclk_div2 or negedge core_rst_n) begin
    if (!core_rst_n) begin
        hdmi_flags_d1 <= 0; hdmi_flags_d2 <= 0;
        ddr_extra_d1 <= 0; ddr_extra_d2 <= 0;
    end else begin
        hdmi_flags_d1 <= hdmi_rx_flags; hdmi_flags_d2 <= hdmi_flags_d1;
        ddr_extra_d1 <= {ddr_writer_fault, ddr_ownership_drop}; ddr_extra_d2 <= ddr_extra_d1;
    end
end
wire			pix_capture_rst_n;
wire			pix_capture_ready;

wire			dma_axis_slave0_tvalid;		
wire	[127:0]	dma_axis_slave0_tdata;		
wire			dma_axis_slave0_tlast;		
wire			dma_axis_slave0_tuser;		

// Reset debounce and sync
wire			sync_button_rst_n; 			
wire			ref_core_rst_n;				
wire			s_pclk_rstn;				

// Internal signal
wire			pclk_div2/*synthesis PAP_MARK_DEBUG="1"*/;  	// 用户时钟，x2 5gt/s时，为125MHZ 2.5gt/s时为62.5
wire			pclk/*synthesis PAP_MARK_DEBUG="1"*/;			// 用户时钟，x2 5gt/s时，为125MHZ 2.5gt/s时为62.5			
wire			ref_clk; 					
wire			core_rst_n;					

wire			axis_master_tvalid;
wire			axis_master_tready;
wire	[127:0]	axis_master_tdata;
wire	[3:0]	axis_master_tkeep;
wire			axis_master_tlast;
wire	[7:0]	axis_master_tuser;

// AXI4-Stream slave 0 interface
wire			axis_slave0_tready;
wire			axis_slave0_tvalid;
wire	[127:0]	axis_slave0_tdata;
wire			axis_slave0_tlast;
wire			axis_slave0_tuser;
// AXI4-Stream slave 1 interface
wire			axis_slave1_tready;
wire			axis_slave1_tvalid;
wire	[127:0]	axis_slave1_tdata;
wire			axis_slave1_tlast;
wire			axis_slave1_tuser;
// AXI4-Stream slave 2 interface
wire			axis_slave2_tready;
wire			axis_slave2_tvalid;
wire	[127:0]	axis_slave2_tdata;
wire			axis_slave2_tlast;
wire			axis_slave2_tuser;

wire	[7:0]	cfg_pbus_num;			
wire	[4:0]	cfg_pbus_dev_num; 		
wire	[2:0]	cfg_max_rd_req_size;	
wire	[2:0]	cfg_max_payload_size;	
wire			cfg_rcb;				

wire			cfg_ido_req_en;			
wire			cfg_ido_cpl_en;			
wire	[7:0]	xadm_ph_cdts;			
wire	[11:0]	xadm_pd_cdts;			
wire	[7:0]	xadm_nph_cdts;			
wire	[11:0]	xadm_npd_cdts;			
wire	[7:0]	xadm_cplh_cdts;			
wire	[11:0]	xadm_cpld_cdts;			

wire	[4:0]	smlh_ltssm_state/*synthesis PAP_MARK_DEBUG="1"*/;//link状态机

// Led lights up signal
reg		[22:0]	ref_led_cnt;		
reg		[26:0]	pclk_led_cnt;		
wire			smlh_link_up; 	
wire			rdlh_link_up/*synthesis PAP_MARK_DEBUG="1"*/; 	

// Independent DDR bring-up status. The vendor BIST owns the DDR user
// interface in this stage; HDMI and PCIe remain disconnected from DDR.
wire			ddr_heart_beat_led;
wire			ddr_err_flag_led;
wire			ddr_uart_txd;
wire			ddr_init_done_status;
wire [9:0]		ddr_init_diag_status;
wire			ddr_error_status;
wire			ddr_bist_running_status;
wire [7:0]		ddr_error_count_status;
wire [3:0]		ddr_test_state_status;
wire			ddr_frame_fifo_overflow;
wire			ddr_frame_ready;
wire [1:0]		ddr_frame_id;
wire [31:0]	ddr_frame_seq;
wire			ddr_frame_writer_error;
wire			ddr_frame_reader_busy;
wire			ddr_frame_reader_valid;
wire [127:0]	ddr_frame_reader_data;
wire			ddr_frame_reader_epoch;
wire			ddr_frame_reader_sof;
wire			ddr_frame_reader_eof;
wire [19:0]	ddr_frame_reader_word_index;
wire			ddr_frame_reader_done;
wire			ddr_frame_reader_error;
wire			ddr_user_clk;
wire			ddr_user_rst_n;
wire			dma_video_wr_ready;
wire			video_source_frame_request;
wire video_source_run, video_source_idle, ddr_dma_fifo_reset_n;
wire			ddr_dma_wr_en;
wire [11:0]	ddr_dma_wr_addr;
wire [127:0]	ddr_dma_wr_data;
wire			ddr_dma_frame_epoch;
wire			ddr_dma_wr_frame_last;
wire			dma_video_wr_clk = DDR_VIDEO_ACTIVE ? ddr_user_clk : capture_clk;
wire			dma_video_wr_rst_n = DDR_VIDEO_ACTIVE ? ddr_dma_fifo_reset_n : pix_capture_rst_n;
wire			dma_video_wr_en = DDR_VIDEO_ACTIVE ? ddr_dma_wr_en : video_wr_en;
wire [11:0]	dma_video_wr_addr = DDR_VIDEO_ACTIVE ? ddr_dma_wr_addr : video_wr_addr;
wire [127:0]	dma_video_wr_data = DDR_VIDEO_ACTIVE ? ddr_dma_wr_data : video_wr_data;
wire			dma_video_frame_epoch = DDR_VIDEO_ACTIVE ? ddr_dma_frame_epoch : video_frame_epoch;
wire			dma_video_wr_frame_last = DDR_VIDEO_ACTIVE ? ddr_dma_wr_frame_last : video_frame_stream_eof;
reg				ddr_heart_beat_ref_d1;
reg				ddr_heart_beat_ref_d2;
reg				ddr_err_flag_pclk_d1;
reg				ddr_err_flag_pclk_d2;
reg				ddr_init_pclk_d1;
reg				ddr_init_pclk_d2;
reg				ddr_error_pclk_d1;
reg				ddr_error_pclk_d2;
reg				ddr_bist_running_pclk_d1;
reg				ddr_bist_running_pclk_d2;
reg [7:0]		ddr_error_count_pclk_d1;
reg [7:0]		ddr_error_count_pclk_d2;
reg [3:0]		ddr_test_state_pclk_d1;
reg [3:0]		ddr_test_state_pclk_d2;
reg [23:0]		ddr_status_timer;
wire			ddr_status_wr_en;
reg [127:0]		ddr_status_wr_data;
reg				hdmi_frame_seen;
reg				ddr_frame_ready_seen;
reg				ddr_reader_done_seen;
wire [31:0] ddr_diag_seq_d2;
reg [7:0]		ddr_diag_flags_d1;
reg [7:0]		ddr_diag_flags_d2;
wire [9:0] ddr_init_diag_pclk_d2;
cdc_snapshot #(.WIDTH(426)) u_ddr_diag_snapshot (
    .src_clk(ddr_user_clk),.src_rst_n(ddr_user_rst_n),
    .src_data({queue_diag_ddr,writer_diag_ddr,writer_fault_diag_ddr,ddr_frame_seq,ddr_init_diag_status}),
    .dst_clk(pclk_div2),.dst_rst_n(core_rst_n),
    .dst_data({queue_diag_pcie,writer_diag_pcie,writer_fault_diag_pcie,ddr_diag_seq_d2,ddr_init_diag_pclk_d2}));
// VID1 v4 retains v3 bits; dedicated pages add counters and atomic snapshots.
reg [1:0] test_ready_pcie;
always @(posedge pclk_div2 or negedge core_rst_n)
    if(!core_rst_n)test_ready_pcie<=0;
    else test_ready_pcie<={test_ready_pcie[0],processed_ready};
wire status_video_live=INTERNAL_VIDEO_TEST ? test_ready_pcie[1] : hdmi_flags_d2[5];
wire [31:0] ddr_frame_buffer_status = {4'd5,
    hdmi_flags_d2[6], hdmi_flags_d2[4], status_video_live, hdmi_flags_d2[7],
    hdmi_flags_d2[3:0], ddr_extra_d2, ddr_init_diag_pclk_d2, ddr_diag_flags_d2};

// Uart to APB 32bits
wire			uart_p_sel;			
wire	[3:0]	uart_p_strb;		
wire	[15:0]	uart_p_addr;		
wire	[31:0]	uart_p_wdata;		
wire			uart_p_ce;			
wire			uart_p_we;			
wire			uart_p_rdy;			
wire	[31:0]	uart_p_rdata;		

// APB signal
wire	[3:0]	p_strb; 			
wire	[15:0]	p_addr; 			
wire	[31:0]	p_wdata; 			
wire			p_ce; 				
wire			p_we; 				

// APB MUX signal
// 0~5: HSSTLP 6: Reserved 7: PCIe
// 8: config
// 9: DMA
wire			p_sel_pcie;			
wire			p_sel_cfg;			
wire			p_sel_dma;			

wire	[31:0]	p_rdata_pcie;		
wire	[31:0]	p_rdata_cfg;		
wire	[31:0]	p_rdata_dma;		

wire			p_rdy_pcie;			
wire			p_rdy_cfg;			
wire			p_rdy_dma;			

assign cfg_ido_req_en	=	1'b0;	
assign cfg_ido_cpl_en	=	1'b0;	
assign xadm_ph_cdts		=	8'b0;	
assign xadm_pd_cdts		=	12'b0;	
assign xadm_nph_cdts	=	8'b0;	
assign xadm_npd_cdts	=	12'b0;	
assign xadm_cplh_cdts	=	8'b0;	
assign xadm_cpld_cdts	=	12'b0;	

// Rst debounce
hsst_rst_cross_sync_v1_0 #(
	.RST_CNTR_VALUE		(16'hC000)
) u_refclk_buttonrstn_debounce (
	.clk				(ref_clk),			
	.rstn_in			(board_power_rst_n), 	
	.rstn_out			(sync_button_rst_n) 
);

hsst_rst_cross_sync_v1_0 #(
	.RST_CNTR_VALUE		(16'hC000)
) u_refclk_perstn_debounce (
	.clk				(ref_clk), 			
	.rstn_in			(board_power_rst_n),			
	.rstn_out			(sync_perst_n)		
);

hsst_rst_sync_v1_0  u_ref_core_rstn_sync (
	.clk				(ref_clk), 			
	.rst_n				(core_rst_n),		
	.sig_async			(1'b1),
	.sig_synced			(ref_core_rst_n)	
);

hsst_rst_sync_v1_0  u_pclk_core_rstn_sync (
	.clk				(pclk),				
	.rst_n				(core_rst_n),		
	.sig_async			(1'b1),
	.sig_synced			(s_pclk_rstn)		
);

// Clk led
always @(posedge ref_clk or negedge sync_perst_n) begin
	if (!sync_perst_n) begin
		ref_led_cnt <= 23'd0;
		ref_led <= 1'b1;
		ddr_heart_beat_ref_d1 <= 1'b0;
		ddr_heart_beat_ref_d2 <= 1'b0;
	end else if (USE_DDR_FRAME_BUFFER) begin
		ddr_heart_beat_ref_d1 <= ddr_heart_beat_led;
		ddr_heart_beat_ref_d2 <= ddr_heart_beat_ref_d1;
		ref_led <= ddr_heart_beat_ref_d2;
	end else if (smlh_link_up & rdlh_link_up) begin
		ref_led_cnt <= ref_led_cnt + 23'd1;
		if(&ref_led_cnt)
			ref_led <= ~ref_led;
	end
end

always @(posedge pclk or negedge s_pclk_rstn) begin
	if (!s_pclk_rstn) begin
		pclk_led_cnt <= 27'd0;
		pclk_led <= 1'b1;
		ddr_err_flag_pclk_d1 <= 1'b0;
		ddr_err_flag_pclk_d2 <= 1'b0;
		ddr_init_pclk_d1 <= 1'b0;
		ddr_init_pclk_d2 <= 1'b0;
		ddr_error_pclk_d1 <= 1'b0;
		ddr_error_pclk_d2 <= 1'b0;
		ddr_bist_running_pclk_d1 <= 1'b0;
		ddr_bist_running_pclk_d2 <= 1'b0;
		ddr_error_count_pclk_d1 <= 8'd0;
		ddr_error_count_pclk_d2 <= 8'd0;
		ddr_test_state_pclk_d1 <= 4'd0;
		ddr_test_state_pclk_d2 <= 4'd0;
		ddr_status_timer <= 24'd0;
	end else if (USE_DDR_FRAME_BUFFER) begin
		ddr_err_flag_pclk_d1 <= ddr_err_flag_led;
		ddr_err_flag_pclk_d2 <= ddr_err_flag_pclk_d1;
		ddr_init_pclk_d1 <= ddr_init_done_status;
		ddr_init_pclk_d2 <= ddr_init_pclk_d1;
		ddr_error_pclk_d1 <= ddr_error_status;
		ddr_error_pclk_d2 <= ddr_error_pclk_d1;
		ddr_bist_running_pclk_d1 <= ddr_bist_running_status;
		ddr_bist_running_pclk_d2 <= ddr_bist_running_pclk_d1;
		ddr_error_count_pclk_d1 <= ddr_error_count_status;
		ddr_error_count_pclk_d2 <= ddr_error_count_pclk_d1;
		ddr_test_state_pclk_d1 <= ddr_test_state_status;
		ddr_test_state_pclk_d2 <= ddr_test_state_pclk_d1;
		ddr_status_timer <= ddr_status_timer + 24'd1;
		// err_flag_led also includes the vendor BIST running indication;
		// pclk_led is reserved for a real sticky DDR error.
		pclk_led <= ddr_error_pclk_d2;
	end else if (smlh_link_up & rdlh_link_up) begin
		pclk_led_cnt <= pclk_led_cnt + 27'd1;
		if(&pclk_led_cnt)
			pclk_led <= ~pclk_led;
	end
end

assign ddr_status_wr_en = USE_DDR_FRAME_BUFFER && !DDR_FRAME_BUFFER_MODE &&
	(ddr_status_timer == 24'hffffff);

// Sticky progress probes make a host timeout actionable without changing the
// video datapath. VID1 BAR0 word2: [0]init [1]HDMI [2]FIFO overflow
// [3]writer error [4]frame ready [5]reader busy [6]reader done [7]reader error.
always @(posedge capture_clk or negedge pix_capture_rst_n) begin
	if (!pix_capture_rst_n)
		hdmi_frame_seen <= 1'b0;
	else if (video_ddr_stream_valid)
		hdmi_frame_seen <= 1'b1;
end

always @(posedge ddr_user_clk or negedge ddr_user_rst_n) begin
	if (!ddr_user_rst_n) begin
		ddr_frame_ready_seen <= 1'b0;
		ddr_reader_done_seen <= 1'b0;
	end else begin
		if (ddr_frame_ready)
			ddr_frame_ready_seen <= 1'b1;
		if (ddr_frame_reader_done)
			ddr_reader_done_seen <= 1'b1;
	end
end

always @(posedge pclk_div2 or negedge core_rst_n) begin
	if (!core_rst_n) begin
		ddr_diag_flags_d1 <= 8'd0;
		ddr_diag_flags_d2 <= 8'd0;
	end else begin
		ddr_diag_flags_d1 <= {ddr_frame_reader_error, ddr_reader_done_seen,
			ddr_frame_reader_busy, ddr_frame_ready_seen, ddr_frame_writer_error,
			ddr_frame_fifo_overflow, hdmi_frame_seen, ddr_init_done_status};
		ddr_diag_flags_d2 <= ddr_diag_flags_d1;
	end
end

always @(*) begin
	ddr_status_wr_data = 128'd0;
	ddr_status_wr_data[31:0] = 32'h44445233; // "DDR3"
	ddr_status_wr_data[39:36] = ddr_test_state_pclk_d2;
	ddr_status_wr_data[40] = ddr_bist_running_pclk_d2;
	ddr_status_wr_data[41] = ddr_error_pclk_d2;
	ddr_status_wr_data[42] = ddr_init_pclk_d2;
	ddr_status_wr_data[50:43] = ddr_error_count_pclk_d2;
end

// UART TO APB
pgr_uart2apb_top_32bit #(
	.CLK_DIV_P		(16'd145)
) u_uart2apb_top (
	.clk			(ref_clk),					
	.rst_n			(ref_core_rst_n),			
	.txd			(txd),						
	.rxd			(rxd),						
	.p_sel			(uart_p_sel),				
	.p_strb			(uart_p_strb),				
	.p_addr			(uart_p_addr),				
	.p_wdata		(uart_p_wdata),				
	.p_ce			(uart_p_ce),				
	.p_we			(uart_p_we),				
	.p_rdy			(uart_p_rdy),				
	.p_rdata		(uart_p_rdata)				
);

// APB MUX
ips2l_expd_apb_mux u_ips2l_pcie_expd_apb_mux (
	// From ref_clk domain
	.i_uart_clk				(ref_clk),			
	.i_uart_rst_n			(ref_core_rst_n),	
	.i_uart_p_sel			(uart_p_sel),		
	.i_uart_p_strb			(uart_p_strb),		
	.i_uart_p_addr			(uart_p_addr),		
	.i_uart_p_wdata			(uart_p_wdata),		
	.i_uart_p_ce			(uart_p_ce),		
	.i_uart_p_we			(uart_p_we),		
	.o_uart_p_rdy			(uart_p_rdy),		
	.o_uart_p_rdata			(uart_p_rdata),		
	// To pclk_div2 clock domain
	.i_pclk_div2_clk		(pclk_div2),		
	.i_pclk_div2_rst_n		(core_rst_n),		

	.o_pclk_div2_p_strb		(p_strb),			
	.o_pclk_div2_p_addr		(p_addr),			
	.o_pclk_div2_p_wdata	(p_wdata),			
	.o_pclk_div2_p_ce		(p_ce),				
	.o_pclk_div2_p_we		(p_we),				

	// To PCIe
	.o_pcie_p_sel			(p_sel_pcie),		
	.i_pcie_p_rdy			(p_rdy_pcie),		
	.i_pcie_p_rdata			(p_rdata_pcie),		

	// To DMA
	.o_dma_p_sel			(p_sel_dma),		
	.i_dma_p_rdy			(p_rdy_dma),		
	.i_dma_p_rdata			(p_rdata_dma),		

	// To config
	.o_cfg_p_sel			(p_sel_cfg),		
	.i_cfg_p_rdy			(p_rdy_cfg),		
	.i_cfg_p_rdata			(p_rdata_cfg)		
);

// DMA CTRL      BASE ADDR = 0x8000
ips2l_pcie_dma #(
	.DEVICE_TYPE			(DEVICE_TYPE),
	.AXIS_SLAVE_NUM			(AXIS_SLAVE_NUM),
	.FRAME_BUFFER_SOURCE	(DDR_VIDEO_ACTIVE),
    .FEATURE_TRAILER_WORDS(FEATURE_TRAILER_WORDS),
    .RX_DIAGNOSTICS(1),.LANE_STATUS(1),.RX_CONTROL(1)
) u_ips2l_pcie_dma (
	.clk					(pclk_div2),				
	.rst_n					(core_rst_n),				

	// HDMI block buffer write port (pixclk_in -> pclk_div2 dual-clock RAM)
	.i_video_wr_clk			(dma_video_wr_clk),
	.i_video_wr_rst_n		(dma_video_wr_rst_n),
	.i_video_wr_en			(dma_video_wr_en),
	.i_video_wr_addr		(dma_video_wr_addr),
	.i_video_wr_data		(dma_video_wr_data),
	.i_video_frame_epoch	(dma_video_frame_epoch),
	.i_video_wr_frame_last	(dma_video_wr_frame_last),
	.o_video_wr_ready		(dma_video_wr_ready),
	.o_video_source_frame_request(video_source_frame_request),
    .o_video_source_run(video_source_run), .i_video_source_idle(video_source_idle),
	.i_external_status_wr_en	(ddr_status_wr_en),
	.i_external_status_wr_data(ddr_status_wr_data),
    .i_lane_status_valid(lane_bar_valid),.o_lane_status_ready(lane_bar_ready),
    .i_lane_status_addr(lane_bar_addr),.i_lane_status_data(lane_bar_data),
	.i_frame_buffer_status	(ddr_frame_buffer_status),
    .i_rx_diag(hdmi_diag_d2), .i_rx_geometry_diag(hdmi_geometry_d2),
    .i_writer_diag(writer_diag_pcie), .i_writer_fault_diag(writer_fault_diag_pcie),
    .i_capture_diag(capture_diag_pcie),
    .i_geometry_fault_diag(geometry_fault_pcie),.i_clock_diag(clock_diag_pcie),.i_timing_diag(timing_diag_pcie),
    .i_low_light_diag(low_light_diag_pcie),
    .i_rx_config_diag(config_diag_pcie),
    .i_rx_control_diag(control_diag_pcie),.i_rx_pad_diag(pad_diag_pcie),
    .o_rx_control_request(hdmi_control_request),
    .i_feature_diag(feature_append_diag_pcie),
    .i_queue_diag(queue_diag_pcie),
	.i_frame_buffer_seq		(ddr_diag_seq_d2),

	// Num
	.i_cfg_pbus_num			(cfg_pbus_num),				
	.i_cfg_pbus_dev_num		(cfg_pbus_dev_num),			
	.i_cfg_max_rd_req_size	(cfg_max_rd_req_size),		
	.i_cfg_max_payload_size	(cfg_max_payload_size),		

	// AXI4-Stream master interface
	.i_axis_master_tvld		(axis_master_tvalid_mem),	
	.o_axis_master_trdy		(axis_master_tready_mem),	
	.i_axis_master_tdata	(axis_master_tdata_mem),	
	.i_axis_master_tkeep	(axis_master_tkeep_mem),	
														
	.i_axis_master_tlast	(axis_master_tlast_mem),	
	.i_axis_master_tuser	(axis_master_tuser_mem),	

	// AXI4-Stream slave0 interface
	.i_axis_slave0_trdy		(axis_slave0_tready),		
	.o_axis_slave0_tvld		(dma_axis_slave0_tvalid),	
	.o_axis_slave0_tdata	(dma_axis_slave0_tdata),	
	.o_axis_slave0_tlast	(dma_axis_slave0_tlast),	
	.o_axis_slave0_tuser	(dma_axis_slave0_tuser),	

	// AXI4-Stream slave1 interface
	.i_axis_slave1_trdy		(axis_slave1_tready),		
	.o_axis_slave1_tvld		(axis_slave1_tvalid),		
	.o_axis_slave1_tdata	(axis_slave1_tdata),		
	.o_axis_slave1_tlast	(axis_slave1_tlast),		
	.o_axis_slave1_tuser	(axis_slave1_tuser),		

	// AXI4-Stream slave2 interface
	.i_axis_slave2_trdy		(axis_slave2_tready),		
	.o_axis_slave2_tvld		(axis_slave2_tvalid),		
	.o_axis_slave2_tdata	(axis_slave2_tdata),		
	.o_axis_slave2_tlast	(axis_slave2_tlast),		
	.o_axis_slave2_tuser	(axis_slave2_tuser),		

	// From pcie
	.i_cfg_ido_req_en		(cfg_ido_req_en),			
	.i_cfg_ido_cpl_en		(cfg_ido_cpl_en),			
	.i_xadm_ph_cdts			(xadm_ph_cdts),				
	.i_xadm_pd_cdts			(xadm_pd_cdts),				
	.i_xadm_nph_cdts		(xadm_nph_cdts),			
	.i_xadm_npd_cdts		(xadm_npd_cdts),			
	.i_xadm_cplh_cdts		(xadm_cplh_cdts),			
	.i_xadm_cpld_cdts		(xadm_cpld_cdts),			

	// APB interface
	.i_apb_psel				(p_sel_dma),				
	.i_apb_paddr			(p_addr[8:0]),				
	.i_apb_pwdata			(p_wdata),					
	.i_apb_pstrb			(p_strb),					
	.i_apb_pwrite			(p_we),						
	.i_apb_penable			(p_ce),						
	.o_apb_prdy				(p_rdy_dma),				
	.o_apb_prdata			(p_rdata_dma),				
	.o_cross_4kb_boundary	(cross_4kb_boundary)		//4k边界
);

hdmi_in_demo u_hdmi_in_demo (
	.sys_clk				(sys_clk),
	.rstn_out				(hdmi_init_rst_n),
	.rx_init_done			(hdmi_rx_init_done),
    .rx_flags(hdmi_rx_flags), .rx_diag(hdmi_rx_diag), .rx_geometry_diag(hdmi_rx_geometry_diag),
    .rx_config_diag(hdmi_config_diag),
    .rx_control_request(control_request_cfg),.rx_control_diag(hdmi_control_diag),.rx_pad_diag(hdmi_pad_diag),
    .rx_cfg_clk(hdmi_cfg_clk),.power_lock_out(board_pll_locked),
    .rx_mode_valid(hdmi_mode_valid),.rx_clock_diag(hdmi_clock_diag),.rx_timing_diag(hdmi_timing_diag),
    .video_vs(video_vs),.video_hs(video_hs),.video_de(video_de),
    .video_r(video_r),.video_g(video_g),.video_b(video_b),
	.hd_scl				(hd_scl),
	.hd_sda				(hd_sda),

	.rx_rst_n                (),
	.pixclk_in				(pixclk_in),
	.vs_in					(vs_in),
	.hs_in					(hs_in),
	.de_in					(de_in),
	.r_in					(r_in),
	.g_in					(g_in),
	.b_in					(b_in)
);

// HDMI init/reset is produced outside pixclk_in.  Deassert it through the
// existing reset synchronizer before it reaches pixclk-domain capture logic.
hsst_rst_cross_sync_v1_0 #(
	.RST_CNTR_WIDTH		(4),
	.RST_CNTR_VALUE		(4'h4)
) u_hdmi_pix_rst_sync (
	.clk					(capture_clk),
	.rstn_in				(hdmi_init_rst_n),
	.rstn_out				(pix_capture_rst_n)
);

// The receiver-ready flag is generated in the HDMI configuration clock
// domain.  Use the existing reset synchronizer as a level CDC bridge before
// capture samples it on the following falling edge.
hsst_rst_cross_sync_v1_0 #(
	.RST_CNTR_WIDTH		(2),
	.RST_CNTR_VALUE		(2'h1)
) u_hdmi_pix_ready_sync (
	.clk					(capture_clk),
	.rstn_in				(hdmi_rx_init_done && hdmi_mode_valid),
	.rstn_out				(pix_capture_ready)
);

// Enhance sampled RGB888 before RGB565 packing. Profile 3 is the lightweight
// causal Retinex path; the legacy profiles remain explicit rollback choices.
generate if(INTERNAL_VIDEO_TEST) begin: g_internal_video_test
    video_pipeline_test_source u_test_source (
        .clk(capture_clk),.rst_n(pix_capture_rst_n),.vs(capture_input_vs),
        .hs(capture_input_hs),.de(capture_input_de),.r(capture_input_r),.g(capture_input_g),.b(capture_input_b));
    assign capture_input_ready=pix_capture_rst_n;
end else begin: g_external_hdmi_input
    assign capture_input_ready=pix_capture_ready;
    assign {capture_input_vs,capture_input_hs,capture_input_de,capture_input_r,capture_input_g,capture_input_b}=
        {video_vs,video_hs,video_de,video_r,video_g,video_b};
end endgenerate
generate if(LOW_LIGHT_PROFILE==3) begin: g_retinex
video_retinex_light u_video_retinex_light (
    .pix_clk(capture_clk),.rst_n(pix_capture_rst_n),.rx_ready(capture_input_ready),
    .vs(capture_input_vs),.hs(capture_input_hs),.de(capture_input_de),.r(capture_input_r),.g(capture_input_g),.b(capture_input_b),
    .mode(LOW_LIGHT_MODE),.manual_strength(LOW_LIGHT_MANUAL_STRENGTH),
    .out_ready(processed_ready),.out_vs(processed_vs),.out_hs(),.out_de(processed_de),
    .out_r(processed_r),.out_g(processed_g),.out_b(processed_b),.diagnostic(low_light_diag_pix));
end else begin: g_legacy_low_light
video_low_light #(.SHADOW_DETAIL(LOW_LIGHT_PROFILE)) u_video_low_light (
    .pix_clk(capture_clk),.rst_n(pix_capture_rst_n),.rx_ready(capture_input_ready),
    .vs(capture_input_vs),.hs(capture_input_hs),.de(capture_input_de),.r(capture_input_r),.g(capture_input_g),.b(capture_input_b),
    .mode(LOW_LIGHT_MODE),.manual_strength(LOW_LIGHT_MANUAL_STRENGTH),
    .out_ready(processed_ready),.out_vs(processed_vs),.out_hs(),.out_de(processed_de),
    .out_r(processed_r),.out_g(processed_g),.out_b(processed_b),.diagnostic(low_light_diag_pix));
end endgenerate

wire lane_bar_valid,lane_bar_ready;
wire cnn_bar_valid,cnn_bar_ready,key_bar_valid,key_bar_ready;
wire [11:0] cnn_bar_addr,key_bar_addr;wire [127:0] cnn_bar_data,key_bar_data;
wire [11:0] lane_bar_addr;wire [127:0] lane_bar_data;
generate if(ENABLE_TINY_CNN)begin:g_tiny_lane
// Raw sampled RGB matches training; the enhanced display path continues independently.
lane_cnn_sidecar_v2 #(.MODEL_VALID(1),.MODEL_VERSION(2),.ROI_CONFIG_VERSION(4),
 .CONV1_SHIFT(8),.CONV2_SHIFT(6),.CONV3_SHIFT(8),.REJECT_MARGIN(1)) u_lane_cnn(
 .video_clk(capture_clk),.video_rst_n(pix_capture_rst_n),.video_ready(capture_input_ready && processed_ready),
 .vs(capture_input_vs),.de(capture_input_de),
 .rgb565({capture_input_r[7:3],capture_input_g[7:2],capture_input_b[7:3]}),
 .source_frame_id(capture_source_frame_id),
 .cnn_clk(hdmi_cfg_clk),.cnn_rst_n(sync_button_rst_n),.pcie_clk(pclk_div2),.pcie_rst_n(core_rst_n),
 .bar_valid(cnn_bar_valid),.bar_ready(cnn_bar_ready),.bar_addr(cnn_bar_addr),.bar_data(cnn_bar_data));
end else begin:g_no_tiny_lane
 assign cnn_bar_valid=0;assign cnn_bar_addr=0;assign cnn_bar_data=0;
end endgenerate

lane_key_bar #(.DEBOUNCE_CYCLES(200000)) u_lane_keys(hdmi_cfg_clk,board_power_rst_n,{key_right_n,key_left_n},pclk_div2,core_rst_n,
 key_bar_valid,key_bar_ready,key_bar_addr,key_bar_data);
lane_status_mux u_lane_status_mux(pclk_div2,core_rst_n,
 cnn_bar_valid,cnn_bar_ready,cnn_bar_addr,cnn_bar_data,key_bar_valid,key_bar_ready,key_bar_addr,key_bar_data,
 lane_bar_valid,lane_bar_ready,lane_bar_addr,lane_bar_data);

video_block_capture #(.INPUT_SAMPLED(1)) u_video_block_capture (
	.pixclk					(capture_clk),
	.rst_n					(pix_capture_rst_n),
	.rx_ready				(processed_ready),
	.vs						(processed_vs),
	.de						(processed_de),
	.r						(processed_r),
	.g						(processed_g),
	.b						(processed_b),
	.wr_en					(video_wr_en),
	.wr_addr				(video_wr_addr),
	.wr_data				(video_wr_data),
	.capture_done			(video_capture_done),
	.frame_epoch			(video_frame_epoch),
	.frame_stream_valid		(video_frame_stream_valid),
	.frame_stream_data		(video_frame_stream_data),
	.frame_stream_epoch		(video_frame_stream_epoch),
	.frame_stream_sof		(video_frame_stream_sof),
	.frame_stream_eof		(video_frame_stream_eof),
	.frame_stream_word_index	(video_frame_stream_word_index),
	.ddr_stream_valid		(video_ddr_stream_valid),
	.ddr_stream_data		(video_ddr_stream_data),
	.ddr_stream_epoch		(video_ddr_stream_epoch),
	.ddr_stream_sof		(video_ddr_stream_sof),
	.ddr_stream_eof		(video_ddr_stream_eof),
	.ddr_stream_word_index	(video_ddr_stream_word_index),
    .ddr_stream_abort(video_ddr_stream_abort), .capture_diag(capture_diag_pix),
    .geometry_fault_diag(geometry_fault_pix)
    ,.source_frame_id_out(capture_source_frame_id),.ddr_stream_source_frame_id(capture_token_source_id)
);

generate if(ENABLE_FPGA_FEATURE) begin: g_feature_transport
    wire stats_valid;
    wire [1023:0] stats_packet;
    // Zero-fill the RGB565 discarded bits so RK can independently recompute
    // exactly the same decisions from the received enhanced image.
    adas_feature_top #(.WIDTH(1920),.HEIGHT(1080),.EXTERNAL_FRAME_ID(1)) u_feature (
        .clk(capture_clk),.rst_n(pix_capture_rst_n),.mode_valid(processed_ready),
        .vs(processed_vs),.de(processed_de),
        .r({processed_r[7:3],3'd0}),.g({processed_g[7:2],2'd0}),.b({processed_b[7:3],3'd0}),
        .source_frame_id(capture_source_frame_id),.feature_valid(feature_packet_valid),
        .feature_packet(feature_packet),.frame_id(),.left_valid(),.right_valid(),.zebra_valid(),
        .left_center_x(),.right_center_x(),.zebra_center_x(),.zebra_center_y());
    adas_road_stats u_road_stats (
        .clk(capture_clk),.rst_n(pix_capture_rst_n),.mode_valid(processed_ready),
        .vs(processed_vs),.de(processed_de),
        .r({processed_r[7:3],3'd0}),.g({processed_g[7:2],2'd0}),.b({processed_b[7:3],3'd0}),
        .source_frame_id(capture_source_frame_id),.stats_valid(stats_valid),.stats_packet(stats_packet));
    video_feature_append #(.LOW_LIGHT_PROFILE(LOW_LIGHT_PROFILE),.LOW_LIGHT_MODE(LOW_LIGHT_MODE),
        .SOURCE_KIND(INTERNAL_VIDEO_TEST),.EXTENDED_ROAD_STATS(1)) u_append (
        .clk(capture_clk),.rst_n(pix_capture_rst_n),
        .in_valid(video_ddr_stream_valid),.in_sof(video_ddr_stream_sof),.in_eof(video_ddr_stream_eof),
        .in_abort(video_ddr_stream_abort),.in_epoch(video_ddr_stream_epoch),
        .in_data(video_ddr_stream_data),.in_index(video_ddr_stream_word_index),.in_source_id(capture_token_source_id),
        .feature_valid(feature_packet_valid),.feature_packet(feature_packet),
        .stats_valid(stats_valid),.stats_packet(stats_packet),
        .out_valid(feature_stream_valid),.out_sof(feature_stream_sof),.out_eof(feature_stream_eof),
        .out_abort(feature_stream_abort),.out_epoch(feature_stream_epoch),.out_data(feature_stream_data),
        .out_index(feature_stream_index),.diagnostic(feature_append_diag));
end else begin: g_without_feature_transport
    assign feature_stream_valid=video_ddr_stream_valid;
    assign feature_stream_sof=video_ddr_stream_sof;
    assign feature_stream_eof=video_ddr_stream_eof;
    assign feature_stream_abort=video_ddr_stream_abort;
    assign feature_stream_epoch=video_ddr_stream_epoch;
    assign feature_stream_data=video_ddr_stream_data;
    assign feature_stream_index=video_ddr_stream_word_index;
    assign feature_packet_valid=0;assign feature_packet=0;assign feature_append_diag=0;
end endgenerate

// Stage 1/2 DDR boundary. The default is the independent BIST. The frame
// buffer branch is explicit so it cannot accidentally replace the validated
// HDMI/PCIe path during bring-up.
generate
	if (USE_DDR_FRAME_BUFFER && DDR_FRAME_BUFFER_MODE) begin : g_ddr_frame_buffer
        reg [3:0] ddr_chunk_slot;
        reg [7:0] ddr_chunk_word;
        wire ddr_reader_request, ddr_reader_cancel, ddr_reader_pending;
        wire ddr_reader_fire=ddr_frame_reader_valid && dma_video_wr_ready;
        video_frame_bridge u_video_frame_bridge (
            .pcie_clk(pclk_div2), .pcie_rst_n(core_rst_n), .i_run(video_source_run),
            .ddr_clk(ddr_user_clk), .ddr_rst_n(ddr_user_rst_n),
            .i_reader_busy(ddr_frame_reader_busy), .i_reader_pending(ddr_reader_pending),
            .o_reader_start(ddr_reader_request), .o_reader_cancel(ddr_reader_cancel),
            .o_source_idle(video_source_idle), .o_fifo_reset_n(ddr_dma_fifo_reset_n));

		always @(posedge ddr_user_clk or negedge ddr_user_rst_n) begin
			if (!ddr_user_rst_n || ddr_reader_cancel) begin
				ddr_chunk_slot <= 4'd0;
				ddr_chunk_word <= 8'd0;
			end else if (ddr_reader_fire) begin
				if (ddr_frame_reader_eof) begin
					ddr_chunk_slot <= 4'd0;
					ddr_chunk_word <= 8'd0;
				end else if (ddr_frame_reader_sof) begin
					ddr_chunk_slot <= 4'd0;
					ddr_chunk_word <= 8'd1;
				end else if (ddr_chunk_word == 8'hff) begin
					ddr_chunk_word <= 8'd0;
					if (ddr_chunk_slot == 4'd14)
						ddr_chunk_slot <= 4'd0;
					else
						ddr_chunk_slot <= ddr_chunk_slot + 4'd1;
				end else begin
					ddr_chunk_word <= ddr_chunk_word + 8'd1;
				end
			end
		end

		assign ddr_dma_wr_en = ddr_reader_fire;
		assign ddr_dma_wr_addr = ddr_frame_reader_sof ? 12'd0 :
			{ddr_chunk_slot, ddr_chunk_word};
		assign ddr_dma_wr_data = ddr_frame_reader_data;
		assign ddr_dma_frame_epoch = ddr_frame_reader_epoch;
		assign ddr_dma_wr_frame_last = ddr_frame_reader_eof;

		ddr_frame_buffer_top #(.FRAME_WORDS(259200+FEATURE_TRAILER_WORDS)) u_ddr_frame_buffer (
			.free_clk				(sys_clk),
			.rst_board			(board_power_rst_n),
			.pix_clk				(capture_clk),
			.pix_rst_n			(pix_capture_rst_n),
			.i_frame_valid		(feature_stream_valid),
			.i_frame_data		(feature_stream_data),
			.i_frame_epoch		(feature_stream_epoch),
			.i_frame_sof			(feature_stream_sof),
			.i_frame_eof			(feature_stream_eof),
            .i_frame_abort(feature_stream_abort),
            .o_writer_diag(writer_diag_ddr), .o_writer_fault_diag(writer_fault_diag_ddr),
            .o_queue_diag(queue_diag_ddr),
			.i_frame_word_index	(feature_stream_index),
			.o_frame_fifo_overflow(ddr_frame_fifo_overflow),
			.o_ddr_init_done		(ddr_init_done_status),
			.o_ddr_init_diag		(ddr_init_diag_status),
			.o_ddr_error			(ddr_error_status),
			.o_ddr_heartbeat		(ddr_heart_beat_led),
			.o_ddr_user_clk		(ddr_user_clk),
			.o_ddr_user_rst_n	(ddr_user_rst_n),
			.o_frame_ready		(ddr_frame_ready),
			.o_frame_id			(ddr_frame_id),
			.o_frame_seq			(ddr_frame_seq),
			.o_frame_writer_error	(ddr_frame_writer_error),
            .o_ownership_drop(ddr_ownership_drop), .o_writer_fault(ddr_writer_fault),
			.i_reader_start		(ddr_reader_request),
            .i_reader_cancel(ddr_reader_cancel), .o_reader_pending(ddr_reader_pending),
			.o_reader_busy		(ddr_frame_reader_busy),
			.o_reader_valid		(ddr_frame_reader_valid),
			.i_reader_ready		(dma_video_wr_ready),
			.o_reader_data		(ddr_frame_reader_data),
			.o_reader_epoch		(ddr_frame_reader_epoch),
			.o_reader_sof			(ddr_frame_reader_sof),
			.o_reader_eof			(ddr_frame_reader_eof),
			.o_reader_word_index	(ddr_frame_reader_word_index),
			.o_reader_done		(ddr_frame_reader_done),
			.o_reader_error		(ddr_frame_reader_error),
			.mem_cs_n			(mem_cs_n),
			.mem_rst_n			(mem_rst_n),
			.mem_ck				(mem_ck),
			.mem_ck_n			(mem_ck_n),
			.mem_cke			(mem_cke),
			.mem_ras_n			(mem_ras_n),
			.mem_cas_n			(mem_cas_n),
			.mem_we_n			(mem_we_n),
			.mem_odt			(mem_odt),
			.mem_a				(mem_a),
			.mem_ba				(mem_ba),
			.mem_dqs			(mem_dqs),
			.mem_dqs_n			(mem_dqs_n),
			.mem_dq				(mem_dq),
			.mem_dm				(mem_dm)
		);
		assign ddr_err_flag_led = ddr_error_status | ddr_frame_fifo_overflow |
			ddr_frame_writer_error | ddr_frame_reader_error;
		assign ddr_uart_txd = 1'b1;
		assign ddr_error_count_status = 8'd0;
		assign ddr_test_state_status = 4'hf;
		assign ddr_bist_running_status = 1'b0;
	end else if (USE_DDR_FRAME_BUFFER) begin : g_ddr_bringup
        assign video_source_idle=1'b1; assign ddr_dma_fifo_reset_n=1'b0;
        assign writer_diag_ddr = 128'd0;
        assign writer_fault_diag_ddr = 128'd0;
        assign queue_diag_ddr = 128'd0;
        assign ddr_ownership_drop = 0;
        assign ddr_writer_fault = 0;
		assign ddr_dma_wr_en = 1'b0;
		assign ddr_dma_wr_addr = 12'd0;
		assign ddr_dma_wr_data = 128'd0;
		assign ddr_dma_frame_epoch = 1'b0;
		assign ddr_dma_wr_frame_last = 1'b0;
		test_ddr u_ddr_bringup (
			.free_clk			(sys_clk),
			.rst_board			(board_power_rst_n),
			.uart_rxd			(1'b1),
			.uart_txd			(ddr_uart_txd),
			.mem_cs_n			(mem_cs_n),
			.mem_rst_n			(mem_rst_n),
			.mem_ck			(mem_ck),
			.mem_ck_n			(mem_ck_n),
			.mem_cke			(mem_cke),
			.mem_ras_n			(mem_ras_n),
			.mem_cas_n			(mem_cas_n),
			.mem_we_n			(mem_we_n),
			.mem_odt		(mem_odt),
			.mem_a				(mem_a),
			.mem_ba			(mem_ba),
			.mem_dqs			(mem_dqs),
			.mem_dqs_n			(mem_dqs_n),
			.mem_dq			(mem_dq),
			.mem_dm			(mem_dm),
			.heart_beat_led		(ddr_heart_beat_led),
			.err_flag_led		(ddr_err_flag_led),
			.ddr_init_done_status(ddr_init_done_status),
			.o_ddr_init_diag	(ddr_init_diag_status),
			.ddr_error_count_status(ddr_error_count_status),
			.ddr_error_status	(ddr_error_status),
			.ddr_test_state_status(ddr_test_state_status),
			.ddr_bist_running_status(ddr_bist_running_status),
			.ddr_user_clk		(ddr_user_clk),
			.ddr_user_rst_n		(ddr_user_rst_n),
			.ext_axi_awaddr		(28'd0),
			.ext_axi_awuser_ap	(1'b0),
			.ext_axi_awuser_id	(4'd0),
			.ext_axi_awlen		(4'd0),
			.ext_axi_awvalid		(1'b0),
			.ext_axi_wdata		(128'd0),
			.ext_axi_wstrb		(16'd0),
			.ext_axi_araddr		(28'd0),
			.ext_axi_aruser_ap	(1'b0),
			.ext_axi_aruser_id	(4'd0),
			.ext_axi_arlen		(4'd0),
			.ext_axi_arvalid		(1'b0)
		);
	end else begin : g_ddr_disabled
        assign video_source_idle=1'b1; assign ddr_dma_fifo_reset_n=1'b0;
        assign writer_diag_ddr = 128'd0;
        assign writer_fault_diag_ddr = 128'd0;
        assign queue_diag_ddr = 128'd0;
		assign ddr_user_clk = sys_clk;
		assign ddr_user_rst_n = board_power_rst_n;
		assign ddr_dma_wr_en = 1'b0;
		assign ddr_dma_wr_addr = 12'd0;
		assign ddr_dma_wr_data = 128'd0;
		assign ddr_dma_frame_epoch = 1'b0;
		assign ddr_dma_wr_frame_last = 1'b0;
		assign mem_cs_n  = 1'b1;
		assign mem_rst_n = 1'b0;
		assign mem_ck    = 1'b0;
		assign mem_ck_n  = 1'b1;
		assign mem_cke   = 1'b0;
		assign mem_ras_n = 1'b1;
		assign mem_cas_n = 1'b1;
		assign mem_we_n  = 1'b1;
		assign mem_odt   = 1'b0;
		assign mem_a     = 15'b0;
		assign mem_ba    = 3'b0;
		assign mem_dqs   = 2'bz;
		assign mem_dqs_n = 2'bz;
		assign mem_dq    = 16'bz;
		assign mem_dm    = 2'b0;
		assign ddr_heart_beat_led = 1'b0;
		assign ddr_err_flag_led   = 1'b0;
		assign ddr_uart_txd       = 1'b1;
		assign ddr_init_done_status = 1'b0;
		assign ddr_init_diag_status = 10'd0;
        assign ddr_ownership_drop = 0;
        assign ddr_writer_fault = 0;
		assign ddr_error_count_status = 8'd0;
		assign ddr_error_status = 1'b0;
		assign ddr_test_state_status = 4'd0;
		assign ddr_bist_running_status = 1'b0;
	end
endgenerate

// CFG CTRL
generate
	if (DEVICE_TYPE == 3'd4) begin:rc
	//CFG TLP TX RX     BASE ADDR = 0x9000
		pcie_cfg_ctrl u_pcie_cfg_ctrl (
			//from APB
			.pclk_div2				(pclk_div2),				//125mhz    x2 5gt/s
			.apb_rst_n				(core_rst_n),				
			.p_sel					(p_sel_cfg),				
			.p_strb					(p_strb),					
			.p_addr					(p_addr[7:0]),				
			.p_wdata				(p_wdata),					
			.p_ce					(p_ce),						
			.p_we					(p_we),						
			.p_rdy					(p_rdy_cfg),				
			.p_rdata				(p_rdata_cfg),				
			.pcie_cfg_ctrl_en		(pcie_cfg_ctrl_en),			

			//To PCIE ctrl
			.axis_slave_tready		(axis_slave0_tready),		
			.axis_slave_tvalid		(cfg_axis_slave0_tvalid),	
			.axis_slave_tlast		(cfg_axis_slave0_tlast),	
			.axis_slave_tuser		(cfg_axis_slave0_tuser),	
			.axis_slave_tdata		(cfg_axis_slave0_tdata),	

			.axis_master_tready		(axis_master_tready_cfg),	
			.axis_master_tvalid		(axis_master_tvalid),		
			.axis_master_tlast		(axis_master_tlast),		

			.axis_master_tkeep		(axis_master_tkeep),		
																

			.axis_master_tdata		(axis_master_tdata)			
		);

		// Logic mux
		assign axis_slave0_tvalid      = pcie_cfg_ctrl_en ? cfg_axis_slave0_tvalid  : dma_axis_slave0_tvalid;
		assign axis_slave0_tlast       = pcie_cfg_ctrl_en ? cfg_axis_slave0_tlast   : dma_axis_slave0_tlast;
		assign axis_slave0_tuser       = pcie_cfg_ctrl_en ? cfg_axis_slave0_tuser   : dma_axis_slave0_tuser;
		assign axis_slave0_tdata       = pcie_cfg_ctrl_en ? cfg_axis_slave0_tdata   : dma_axis_slave0_tdata;

		assign axis_master_tvalid_mem  = pcie_cfg_ctrl_en ? 1'b0                    : axis_master_tvalid;
		assign axis_master_tdata_mem   = pcie_cfg_ctrl_en ? 128'b0                  : axis_master_tdata;
		assign axis_master_tkeep_mem   = pcie_cfg_ctrl_en ? 4'b0                    : axis_master_tkeep;
		assign axis_master_tlast_mem   = pcie_cfg_ctrl_en ? 1'b0                    : axis_master_tlast;
		assign axis_master_tuser_mem   = pcie_cfg_ctrl_en ? 8'b0                    : axis_master_tuser;

		assign axis_master_tready      = pcie_cfg_ctrl_en ? axis_master_tready_cfg  : axis_master_tready_mem;
	end else begin:ep
		assign p_rdy_cfg               = 1'b0;
		assign p_rdata_cfg             = 32'b0;

		assign axis_slave0_tvalid      = dma_axis_slave0_tvalid;
		assign axis_slave0_tlast       = dma_axis_slave0_tlast;
		assign axis_slave0_tuser       = dma_axis_slave0_tuser;
		assign axis_slave0_tdata       = dma_axis_slave0_tdata;

		assign axis_master_tvalid_mem  = axis_master_tvalid;
		assign axis_master_tdata_mem   = axis_master_tdata;
		assign axis_master_tkeep_mem   = axis_master_tkeep;
		assign axis_master_tlast_mem   = axis_master_tlast;
		assign axis_master_tuser_mem   = axis_master_tuser;

		assign axis_master_tready      = axis_master_tready_mem;
	end
endgenerate

// PCIe IP TOP : HSSTLP : 0x0000~6000 PCIe BASE ADDR : 0x7000
pcie_test u_ips2l_pcie_wrap (
	.button_rst_n				(sync_button_rst_n),	
	.power_up_rst_n				(sync_perst_n),			
	.perst_n					(sync_perst_n),			

	// The clock and reset signals
	.pclk						(pclk),					
	.pclk_div2					(pclk_div2),			
	.ref_clk					(ref_clk),				
	.ref_clk_n					(ref_clk_n),			
	.ref_clk_p					(ref_clk_p),			
	.core_rst_n					(core_rst_n),			

	// APB interface to DBI config
	.p_sel						(p_sel_pcie),			
	.p_strb						(uart_p_strb),			
	.p_addr						(uart_p_addr),			
	.p_wdata					(uart_p_wdata),			
	.p_ce						(uart_p_ce),			
	.p_we						(uart_p_we),			
	.p_rdy						(p_rdy_pcie),			
	.p_rdata					(p_rdata_pcie),			

	// PHY diff signals
	.rxn						(rxn),					
	.rxp						(rxp),					
	.txn						(txn),					
	.txp						(txp),					
	.pcs_nearend_loop			({4{1'b0}}),			
	.pma_nearend_ploop			({4{1'b0}}),			
	.pma_nearend_sloop			({4{1'b0}}),			

	// AXI4-Stream master interface
	.axis_master_tvalid			(axis_master_tvalid),	
	.axis_master_tready			(axis_master_tready),	
	.axis_master_tdata			(axis_master_tdata),	
	.axis_master_tkeep			(axis_master_tkeep),	
														
	.axis_master_tlast			(axis_master_tlast),	
	.axis_master_tuser			(axis_master_tuser),	

	// AXI4-Stream slave 0 interface
	.axis_slave0_tready			(axis_slave0_tready),	
	.axis_slave0_tvalid			(axis_slave0_tvalid),	
	.axis_slave0_tdata			(axis_slave0_tdata),	
	.axis_slave0_tlast			(axis_slave0_tlast),	
	.axis_slave0_tuser			(axis_slave0_tuser),	

	// AXI4-Stream slave 1 interface
	.axis_slave1_tready			(axis_slave1_tready),	
	.axis_slave1_tvalid			(axis_slave1_tvalid),	
	.axis_slave1_tdata			(axis_slave1_tdata),	
	.axis_slave1_tlast			(axis_slave1_tlast),	
	.axis_slave1_tuser			(axis_slave1_tuser),	

	// AXI4-Stream slave 2 interface
	.axis_slave2_tready			(axis_slave2_tready),	
	.axis_slave2_tvalid			(axis_slave2_tvalid),	
	.axis_slave2_tdata			(axis_slave2_tdata),	
	.axis_slave2_tlast			(axis_slave2_tlast),	
	.axis_slave2_tuser			(axis_slave2_tuser),	

	.pm_xtlh_block_tlp			(),						

	.cfg_send_cor_err_mux		(),						
	.cfg_send_nf_err_mux		(),						
	.cfg_send_f_err_mux			(),						
	.cfg_sys_err_rc				(),						
	.cfg_aer_rc_err_mux			(),						

	// The radm timeout
	.radm_cpl_timeout			(),						

	// Configuration signals
	.cfg_max_rd_req_size		(cfg_max_rd_req_size),	
	.cfg_bus_master_en			(),						
	.cfg_max_payload_size		(cfg_max_payload_size),	
	.cfg_ext_tag_en				(),						
	.cfg_rcb					(cfg_rcb),				
	.cfg_mem_space_en			(),						
	.cfg_pm_no_soft_rst			(),						
	.cfg_crs_sw_vis_en			(),						
	.cfg_no_snoop_en			(),						
	.cfg_relax_order_en			(),						
	.cfg_tph_req_en				(),						
	.cfg_pf_tph_st_mode			(),						
	.rbar_ctrl_update			(),						
	.cfg_atomic_req_en			(),						

	.cfg_pbus_num				(cfg_pbus_num),			
	.cfg_pbus_dev_num			(cfg_pbus_dev_num),		

	// Debug signals
	.radm_idle					(),						
	.radm_q_not_empty			(),						
	.radm_qoverflow				(),						
	.diag_ctrl_bus				(2'b0),					
	.cfg_link_auto_bw_mux		(),						
	.cfg_bw_mgt_mux				(),						
	.cfg_pme_mux				(),						
	.app_ras_des_sd_hold_ltssm	(1'b0),					
	.app_ras_des_tba_ctrl		(2'b0),					

	.dyn_debug_info_sel			(4'b0),					
	.debug_info_mux				(),

	// System signal
	.smlh_link_up				(smlh_link_up),			//link状态
	.rdlh_link_up				(rdlh_link_up),			//link状态
	.smlh_ltssm_state			(smlh_ltssm_state)
);

endmodule
