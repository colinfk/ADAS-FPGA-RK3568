`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company:Meyesemi 
// Engineer: 1111
// 
// Create Date: 2026-04-5 9:35  
// Design Name:  
// Module Name: 
// Project Name: 
// Target Devices: Pango
// Tool Versions: 
// Description: 
//      
// Dependencies: 
// 
// Revision:
// Revision 1.0 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
`define UD #1

module hdmi_in_demo(
	input  wire                         sys_clk                    ,
	output                              rstn_out                   ,
	output wire                         rx_init_done               ,
    output wire [7:0] rx_flags,
    output wire [127:0] rx_diag, rx_geometry_diag, rx_config_diag,
    output wire rx_cfg_clk,
    output wire power_lock_out,
    output wire rx_mode_valid,
    output wire [127:0] rx_clock_diag, rx_timing_diag,
    input wire [31:0] rx_control_request,
    output wire [127:0] rx_control_diag, rx_pad_diag,
    output wire video_vs, video_hs, video_de,
    output wire [7:0] video_r, video_g, video_b,

//HDMI_IN
    output                              hd_scl                     ,
    inout                               hd_sda                     ,
    output                              rx_rst_n                   ,
    input                               pixclk_in /*synthesis PAP_MARK_DEBUG="true"*/                 ,
    input                               vs_in     /*synthesis PAP_MARK_DEBUG="true"*/                 ,
    input                               hs_in     /*synthesis PAP_MARK_DEBUG="true"*/                 ,
    input                               de_in     /*synthesis PAP_MARK_DEBUG="true"*/                 ,
    input                [   7: 0]      r_in      /*synthesis PAP_MARK_DEBUG="true"*/                 ,
    input                [   7: 0]      g_in      /*synthesis PAP_MARK_DEBUG="true"*/                 ,
    input                [   7: 0]      b_in      /*synthesis PAP_MARK_DEBUG="true"*/                  


);

    wire                                cfg_clk                    ;
    assign rx_cfg_clk=cfg_clk;
    wire                                locked                     ;
    assign power_lock_out=locked;
    wire                                init_over                  ;
    wire reconfigure, rx_error_seen;
    wire [15:0] rx_error_count, rx_error_addr, recovery_count;
    wire [31:0] rx_freq_rec;
    wire [7:0] rx_phase;
    wire [4:0] rx_state;
    wire [7:0] monitor_flags;
    wire sample_rising;
    reg                  [  15: 0]      rstn_1ms                   ;

PLL_CLK u_PLL_CLK (
    .clkout0                            (cfg_clk                   ),// output
    .lock                               (locked                    ),// output
    .clkin1                             (sys_clk                   ) // input
);

ms72xx_ctl ms72xx_ctl(
    .reconfigure(reconfigure),
    .video_valid(monitor_flags[5] && rx_mode_valid), .config_diag(rx_config_diag),
    .control_request(rx_control_request),.control_diag(rx_control_diag),.sample_rising(sample_rising),
    .error_seen(rx_error_seen), .error_count(rx_error_count),
    .last_error_addr(rx_error_addr), .freq_rec(rx_freq_rec),
    .phase_readback(rx_phase), .debug_state(rx_state),
    .clk                                (cfg_clk                   ),//input       clk,
    .rst_n                              (rstn_out                  ),//input       rstn,
           
	    .init_over_rx                       (rx_init_done              ),
    .init_over                          (init_over                 ),//output      init_over,
    .iic_scl                            (hd_scl                    ),//output      iic_scl,
    .iic_sda                            (hd_sda                    ) //inout       iic_sda
);

always @(posedge cfg_clk)
begin
	if(!locked)
	    rstn_1ms <= 16'd0;
	else
	begin
		if(rstn_1ms == 16'h2710)
		    rstn_1ms <= rstn_1ms;
		else
		    rstn_1ms <= rstn_1ms + 1'b1;
	end
end
assign rstn_out = (rstn_1ms == 16'h2710);
// rx_rst_n is an internal compatibility port only. It must not become an
// unconstrained pad on the MES_40P_HDMI V1.1 connector.
assign rx_rst_n = rstn_out;
hdmi_input_sampler u_input_sampler (
    .sample_rising(sample_rising),
    .pix_clk(pixclk_in),.vs(vs_in),.hs(hs_in),.de(de_in),.r(r_in),.g(g_in),.b(b_in),
    .vs_q(video_vs),.hs_q(video_hs),.de_q(video_de),.r_q(video_r),.g_q(video_g),.b_q(video_b));
hdmi_rx_monitor #(.INPUT_SAMPLED(1)) u_rx_monitor (
    .cfg_clk(cfg_clk), .pix_clk(pixclk_in), .rst_n(rstn_out), .init_done(rx_init_done),
    .vs(video_vs), .hs(video_hs), .de(video_de), .r(video_r), .g(video_g), .b(video_b),
    .reconfigure(reconfigure), .flags(monitor_flags),
    .geometry_diag(rx_geometry_diag), .recovery_count(recovery_count)
);
hdmi_timing_monitor u_timing_monitor (
    .cfg_clk(cfg_clk),.pix_clk(pixclk_in),.rst_n(rstn_out),.init_done(rx_init_done),
    .vs(video_vs),.hs(video_hs),.de(video_de),.mode_valid(rx_mode_valid),
    .clock_diag(rx_clock_diag),.timing_diag(rx_timing_diag));
assign rx_flags = {rx_error_seen, monitor_flags[6],monitor_flags[5] && rx_mode_valid,monitor_flags[4:0]};
assign rx_diag = {{3'd0, recovery_count, rx_state, rx_phase}, rx_freq_rec,
                  rx_error_count, rx_error_addr, 32'h52584431};
// Independent low-rate pad telemetry separates receiver output from pixel sampling.
reg [2:0] pad_sync1, pad_sync2, pad_previous;
reg [31:0] pad_window, pad_vs_edges, pad_hs_edges, pad_de_edges;
reg [95:0] pad_counts;
always @(posedge cfg_clk or negedge rstn_out) begin
    if (!rstn_out) begin
        pad_sync1<=0; pad_sync2<=0; pad_previous<=0;
        pad_window<=0; pad_vs_edges<=0; pad_hs_edges<=0; pad_de_edges<=0; pad_counts<=0;
    end else begin
        pad_sync1<={de_in,hs_in,vs_in}; pad_sync2<=pad_sync1; pad_previous<=pad_sync2;
        if (pad_window==999999) begin
            pad_counts<={pad_de_edges,pad_hs_edges,pad_vs_edges};
            pad_window<=0;
            pad_vs_edges<=pad_sync2[0]&&!pad_previous[0];
            pad_hs_edges<=pad_sync2[1]&&!pad_previous[1];
            pad_de_edges<=pad_sync2[2]&&!pad_previous[2];
        end else begin
            pad_window<=pad_window+1'b1;
            if (pad_sync2[0]&&!pad_previous[0]) pad_vs_edges<=pad_vs_edges+1'b1;
            if (pad_sync2[1]&&!pad_previous[1]) pad_hs_edges<=pad_hs_edges+1'b1;
            if (pad_sync2[2]&&!pad_previous[2]) pad_de_edges<=pad_de_edges+1'b1;
        end
    end
end
assign rx_pad_diag={pad_counts,32'h52584831};

endmodule

// Frontend modules share this existing PDS source so open projects load them.

// One source-synchronous pad sampler for both capture and telemetry.
// No reset/enable on the pad registers: ADS can pack them into I/O registers.
// Controls and all RGB bits have identical latency. Qualification/reset is
// applied by consumers. The existing falling edge is retained deliberately.
module hdmi_input_sampler (
    input pix_clk, input vs, hs, de, input [7:0] r, g, b,
    input sample_rising,
    output reg vs_q, hs_q, de_q, output reg [7:0] r_q, g_q, b_q
);
reg [26:0] pad_q /* synthesis syn_useioff=1 syn_preserve=1 */;
reg [26:0] rising_q /* synthesis syn_preserve=1 */;
reg select_sync1, select_sync2;
always @(negedge pix_clk) pad_q <= {vs, hs, de, r, g, b};
always @(posedge pix_clk) begin
    rising_q <= {vs,hs,de,r,g,b};
    select_sync1 <= sample_rising === 1'b1;
    select_sync2 <= select_sync1;
end
// Only this bank loads pad_q. Any later fanout replication is downstream of
// the shared physical sampler and cannot sample the pad at different times.
always @(posedge pix_clk) {vs_q,hs_q,de_q,r_q,g_q,b_q} <= select_sync2 ? rising_q : pad_q;
endmodule

// Measure timing independently of DMA. Pixel counters cross as a coherent
// snapshot; VS crosses as a toggle, so a stopped input cannot leave mode valid.
module hdmi_timing_monitor #(
    parameter integer CFG_HZ=10000000,
    parameter integer EXPECTED_PCLK=74250000,
    parameter integer WINDOW_CYCLES=CFG_HZ/10,
    parameter integer EXPECTED_HS_PIXELS=2200,
    parameter integer EXPECTED_VS_PIXELS=2475000,
    parameter integer GOOD_FRAMES=3
) (
    input cfg_clk, pix_clk, rst_n, init_done, input vs, hs, de,
    output mode_valid,
    output [127:0] clock_diag, timing_diag
);
reg [31:0] pixel_ticks, vs_ticks, hs_ticks, frame_pixels, line_pixels, overlap;
reg vs_d, hs_d, vs_toggle, vs_seen, hs_seen;
reg [31:0] frame_sequence;
reg frame_valid, frame_bad;
reg [1:0] pix_reset;
always @(posedge pix_clk or negedge rst_n)
    if (!rst_n) pix_reset<=0; else pix_reset<={pix_reset[0],1'b1};
wire pix_local_rst_n=pix_reset[1];
always @(posedge pix_clk or negedge pix_local_rst_n) begin
    if (!pix_local_rst_n) begin
        pixel_ticks<=0; vs_ticks<=0; hs_ticks<=0; frame_pixels<=0;
        line_pixels<=0; overlap<=0; vs_d<=0; hs_d<=0;
        vs_toggle<=0; vs_seen<=0; hs_seen<=0;
        frame_sequence<=0; frame_valid<=0; frame_bad<=0;
    end else begin
        pixel_ticks<=pixel_ticks+1'b1;
        if (pix_reset[1]) begin
            vs_d<=vs; hs_d<=hs;
            if (de && vs) begin
                frame_bad<=1;
                if (overlap!=32'hffffffff) overlap<=overlap+1'b1;
            end
            if (vs && !vs_d) begin
                vs_ticks<=pixel_ticks; vs_seen<=1; vs_toggle<=~vs_toggle;
                if (vs_seen) frame_pixels<=pixel_ticks-vs_ticks;
                frame_sequence<=frame_sequence+1'b1;
                frame_valid<=vs_seen && hs_seen && !frame_bad && !de &&
                    pixel_ticks-vs_ticks==EXPECTED_VS_PIXELS &&
                    line_pixels==EXPECTED_HS_PIXELS;
                frame_bad<=de;
            end
            if (hs && !hs_d) begin
                hs_ticks<=pixel_ticks; hs_seen<=1;
                if (hs_seen) begin
                    line_pixels<=pixel_ticks-hs_ticks;
                    if (pixel_ticks-hs_ticks!=EXPECTED_HS_PIXELS) frame_bad<=1;
                end
            end
        end
    end
end
wire [161:0] timing_snapshot;
wire [127:0] pix_snapshot=timing_snapshot[127:0];
wire [31:0] snapshot_frame=timing_snapshot[161:130];
wire snapshot_frame_valid=timing_snapshot[129];
wire snapshot_frame_bad=timing_snapshot[128];
cdc_snapshot #(.WIDTH(162)) u_timing_snapshot (
    .src_clk(pix_clk),.src_rst_n(pix_local_rst_n),
    .src_data({frame_sequence,frame_valid,frame_bad,pixel_ticks,frame_pixels,line_pixels,overlap}),
    .dst_clk(cfg_clk),.dst_rst_n(rst_n),.dst_data(timing_snapshot));
reg vs_sync1,vs_sync2,vs_sync3;
reg [31:0] window_count, previous_ticks, measured_hz, vs_age, vs_period;
reg clock_measured, period_measured, cfg_vs_seen;
reg [31:0] qualified_frame;
reg [3:0] stable_frames;
wire vs_event=vs_sync2^vs_sync3;
wire clock_ok=clock_measured && measured_hz>=(EXPECTED_PCLK/100)*97 &&
              measured_hz<=(EXPECTED_PCLK/100)*103;
wire period_ok=period_measured && vs_period>=CFG_HZ/31 && vs_period<=CFG_HZ/29;
wire hs_ok=pix_snapshot[63:32]==EXPECTED_HS_PIXELS;
wire vs_pixels_ok=pix_snapshot[95:64]==EXPECTED_VS_PIXELS;
wire sync_clean=!snapshot_frame_bad;
assign mode_valid=init_done && clock_ok && period_ok && hs_ok && vs_pixels_ok &&
    sync_clean && stable_frames>=GOOD_FRAMES && vs_age<CFG_HZ/20;
always @(posedge cfg_clk or negedge rst_n) begin
    if (!rst_n) begin
        vs_sync1<=0;vs_sync2<=0;vs_sync3<=0;
        window_count<=0; previous_ticks<=0; measured_hz<=0;
        vs_age<=CFG_HZ; vs_period<=0; clock_measured<=0;
        period_measured<=0; cfg_vs_seen<=0;
        qualified_frame<=0; stable_frames<=0;
    end else begin
        vs_sync1<=vs_toggle;vs_sync2<=vs_sync1;vs_sync3<=vs_sync2;
        if (vs_event) begin
            vs_age<=0; cfg_vs_seen<=1;
            if (cfg_vs_seen) begin vs_period<=vs_age+1'b1; period_measured<=1; end
        end else if (vs_age<CFG_HZ) vs_age<=vs_age+1'b1;
        if (!init_done) begin cfg_vs_seen<=0; period_measured<=0; end
        if (!init_done || !clock_ok || !period_ok || !sync_clean || vs_age>=CFG_HZ/20) begin
            stable_frames<=0; qualified_frame<=snapshot_frame;
        end else if (snapshot_frame!=qualified_frame) begin
            qualified_frame<=snapshot_frame;
            if (snapshot_frame_valid && snapshot_frame-qualified_frame==1 && hs_ok && vs_pixels_ok) begin
                if (stable_frames<GOOD_FRAMES) stable_frames<=stable_frames+1'b1;
            end else stable_frames<=0;
        end
        if (window_count==WINDOW_CYCLES-1) begin
            window_count<=0; previous_ticks<=pix_snapshot[127:96];
            measured_hz<=(pix_snapshot[127:96]-previous_ticks)*(CFG_HZ/WINDOW_CYCLES);
            clock_measured<=1;
        end else window_count<=window_count+1'b1;
    end
end
// CLK1: flags, VS period in configuration ticks, PCLK Hz. TIM1: cumulative
// DE/VS overlap cycles, HS period in pixels, VS period in pixels.
assign clock_diag={measured_hz,vs_period,{8'd7,8'd0,stable_frames,3'd0,sync_clean,vs_pixels_ok,hs_ok,init_done,period_measured,
                  clock_measured,period_ok,clock_ok,mode_valid},32'h434c4b31};
assign timing_diag={pix_snapshot[95:64],pix_snapshot[63:32],pix_snapshot[31:0],32'h54494d31};
endmodule
