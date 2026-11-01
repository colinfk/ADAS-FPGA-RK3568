`timescale 1ns / 1ps
// RX-only sequencer. Vendor receiver settings with a preferred RGB 1080p30 EDID.
// EDID writes are generated/checked by make_edid_1080p30.py.
// Each transaction holds its command until busy rises and falls. A NACK must
// never advance the table or assert init_over. Addresses are byte-ordered by
// iic_dri exactly as in the original MS7200 interface (low address byte first).
module ms7200_ctl #(
    parameter integer CLK_FRE = 10000000,
    parameter integer POLL_CYCLES = CLK_FRE / 10,
    parameter integer RETRY_CYCLES = CLK_FRE / 100,
    parameter integer TX_TIMEOUT_CYCLES = CLK_FRE / 20,
    parameter integer MAX_RETRIES = 3,
    parameter integer SETTLE_CYCLES = CLK_FRE / 20,
    parameter integer RECOVERY_COOLDOWN_CYCLES = CLK_FRE * 5,
    parameter integer CONFIG_STEP_CYCLES = CLK_FRE / 50
)(
    input clk, input rstn,
    output reg init_over, output [7:0] device_id,
    output reg iic_trig, output reg w_r,
    output reg [15:0] addr, output reg [7:0] data_in,
    input busy, input [7:0] data_out, input byte_over,
    input txn_error, input reconfigure, input video_valid,
    output reg iic_abort,
    output reg error_seen, output reg [15:0] error_count,
    output reg [15:0] last_error_addr,
    output reg [31:0] freq_rec,
    output reg [7:0] phase_readback,
    output [4:0] debug_state,
    output [127:0] config_diag,
    input [31:0] control_request,
    output [127:0] control_diag,
    output reg sample_rising
);
assign device_id = 8'h56;
function [23:0] cmd_data;
input [9:0] index;
    begin
        case(index)
            9'd0   : cmd_data = {16'h0204, 8'h02};
            9'd1   : cmd_data = {16'h0205, 8'h40};
            9'd2   : cmd_data = {16'h0004, 8'h01};
            9'd3   : cmd_data = {16'h0009, 8'h26};
            9'd4   : cmd_data = {16'h102E, 8'h01};
            9'd5   : cmd_data = {16'h1025, 8'hB0};
            9'd6   : cmd_data = {16'h102D, 8'h83};
            9'd7   : cmd_data = {16'h1000, 8'h20};
            9'd8   : cmd_data = {16'h100F, 8'h04};
            9'd9   : cmd_data = {16'h0003, 8'hC3};
            9'd10  : cmd_data = {16'h103B, 8'h02};
            9'd11  : cmd_data = {16'h00E1, 8'h00};
            9'd12  : cmd_data = {16'h00A8, 8'h08};
            9'd13  : cmd_data = {16'h000A, 8'h00};
            9'd14  : cmd_data = {16'h0016, 8'h02};
            9'd15  : cmd_data = {16'h00E2, 8'h01};
            9'd16  : cmd_data = {16'h00C2, 8'h14};
            9'd17  : cmd_data = {16'h102D, 8'hC7};
            9'd18  : cmd_data = {16'h1005, 8'h04};
            9'd19  : cmd_data = {16'h1003, 8'h14};
            9'd20  : cmd_data = {16'h1004, 8'h01};
            9'd21  : cmd_data = {16'h1000, 8'h60};
            9'd22  : cmd_data = {16'h1020, 8'h13};
            9'd23  : cmd_data = {16'h1021, 8'h04};
            9'd24  : cmd_data = {16'h1022, 8'h04};
            9'd25  : cmd_data = {16'h1023, 8'h0C};
            9'd26  : cmd_data = {16'h102B, 8'h33};
            9'd27  : cmd_data = {16'h102C, 8'h33};
            9'd28  : cmd_data = {16'h00F0, 8'h10};
            9'd29  : cmd_data = {16'h000C, 8'h80};
            9'd30  : cmd_data = {16'h0D00, 8'h00};
            9'd31  : cmd_data = {16'h0D01, 8'hFF};
            9'd32  : cmd_data = {16'h0D02, 8'hFF};
            9'd33  : cmd_data = {16'h0D03, 8'hFF};
            9'd34  : cmd_data = {16'h0D04, 8'hFF};
            9'd35  : cmd_data = {16'h0D05, 8'hFF};
            9'd36  : cmd_data = {16'h0D06, 8'hFF};
            9'd37  : cmd_data = {16'h0D07, 8'h00};
            9'd38  : cmd_data = {16'h0D00, 8'h00};
            9'd39  : cmd_data = {16'h0D01, 8'hFF};
            9'd40  : cmd_data = {16'h0D02, 8'hFF};
            9'd41  : cmd_data = {16'h0D03, 8'hFF};
            9'd42  : cmd_data = {16'h0D04, 8'hFF};
            9'd43  : cmd_data = {16'h0D05, 8'hFF};
            9'd44  : cmd_data = {16'h0D06, 8'hFF};
            9'd45  : cmd_data = {16'h0D07, 8'h00};
            9'd46  : cmd_data = {16'h0D08, 8'h4C};
            9'd47  : cmd_data = {16'h0D09, 8'h2D};
            9'd48  : cmd_data = {16'h0D0A, 8'hFF};
            9'd49  : cmd_data = {16'h0D0B, 8'h0D};
            9'd50  : cmd_data = {16'h0D0C, 8'h58};
            9'd51  : cmd_data = {16'h0D0D, 8'h4D};
            9'd52  : cmd_data = {16'h0D0E, 8'h51};
            9'd53  : cmd_data = {16'h0D0F, 8'h30};
            9'd54  : cmd_data = {16'h0D10, 8'h1C};
            9'd55  : cmd_data = {16'h0D11, 8'h1C};
            9'd56  : cmd_data = {16'h0D12, 8'h01};
            9'd57  : cmd_data = {16'h0D13, 8'h03};
            9'd58  : cmd_data = {16'h0D14, 8'h80};
            9'd59  : cmd_data = {16'h0D15, 8'h00};
            9'd60  : cmd_data = {16'h0D16, 8'h00};
            9'd61  : cmd_data = {16'h0D17, 8'h78};
            9'd62  : cmd_data = {16'h0D18, 8'h0A};
            9'd63  : cmd_data = {16'h0D19, 8'h5F};
            9'd64  : cmd_data = {16'h0D1A, 8'hB1};
            9'd65  : cmd_data = {16'h0D1B, 8'hA2};
            9'd66  : cmd_data = {16'h0D1C, 8'h57};
            9'd67  : cmd_data = {16'h0D1D, 8'h4F};
            9'd68  : cmd_data = {16'h0D1E, 8'hA2};
            9'd69  : cmd_data = {16'h0D1F, 8'h28};
            9'd70  : cmd_data = {16'h0D20, 8'h0F};
            9'd71  : cmd_data = {16'h0D21, 8'h50};
            9'd72  : cmd_data = {16'h0D22, 8'h54};
            9'd73  : cmd_data = {16'h0D23, 8'h20};
            9'd74  : cmd_data = {16'h0D24, 8'h00};
            9'd75  : cmd_data = {16'h0D25, 8'h00};
            9'd76  : cmd_data = {16'h0D26, 8'h01};
            9'd77  : cmd_data = {16'h0D27, 8'h01};
            9'd78  : cmd_data = {16'h0D28, 8'h01};
            9'd79  : cmd_data = {16'h0D29, 8'h01};
            9'd80  : cmd_data = {16'h0D2A, 8'h01};
            9'd81  : cmd_data = {16'h0D2B, 8'h01};
            9'd82  : cmd_data = {16'h0D2C, 8'h01};
            9'd83  : cmd_data = {16'h0D2D, 8'h01};
            9'd84  : cmd_data = {16'h0D2E, 8'h01};
            9'd85  : cmd_data = {16'h0D2F, 8'h01};
            9'd86  : cmd_data = {16'h0D30, 8'h01};
            9'd87  : cmd_data = {16'h0D31, 8'h01};
            9'd88  : cmd_data = {16'h0D32, 8'h01};
            9'd89  : cmd_data = {16'h0D33, 8'h01};
            9'd90  : cmd_data = {16'h0D34, 8'h01};
            9'd91  : cmd_data = {16'h0D35, 8'h01};
            9'd92  : cmd_data = {16'h0D36, 8'h01};
            9'd93  : cmd_data = {16'h0D37, 8'h1D};
            9'd94  : cmd_data = {16'h0D38, 8'h80};
            9'd95  : cmd_data = {16'h0D39, 8'h18};
            9'd96  : cmd_data = {16'h0D3A, 8'h71};
            9'd97  : cmd_data = {16'h0D3B, 8'h38};
            9'd98  : cmd_data = {16'h0D3C, 8'h2D};
            9'd99  : cmd_data = {16'h0D3D, 8'h40};
            9'd100 : cmd_data = {16'h0D3E, 8'h58};
            9'd101 : cmd_data = {16'h0D3F, 8'h2C};
            9'd102 : cmd_data = {16'h0D40, 8'h45};
            9'd103 : cmd_data = {16'h0D41, 8'h00};
            9'd104 : cmd_data = {16'h0D42, 8'h00};
            9'd105 : cmd_data = {16'h0D43, 8'h00};
            9'd106 : cmd_data = {16'h0D44, 8'h00};
            9'd107 : cmd_data = {16'h0D45, 8'h00};
            9'd108 : cmd_data = {16'h0D46, 8'h00};
            9'd109 : cmd_data = {16'h0D47, 8'h1E};
            9'd110 : cmd_data = {16'h0D48, 8'h00};
            9'd111 : cmd_data = {16'h0D49, 8'h00};
            9'd112 : cmd_data = {16'h0D4A, 8'h00};
            9'd113 : cmd_data = {16'h0D4B, 8'hFC};
            9'd114 : cmd_data = {16'h0D4C, 8'h00};
            9'd115 : cmd_data = {16'h0D4D, 8'h41};
            9'd116 : cmd_data = {16'h0D4E, 8'h44};
            9'd117 : cmd_data = {16'h0D4F, 8'h41};
            9'd118 : cmd_data = {16'h0D50, 8'h53};
            9'd119 : cmd_data = {16'h0D51, 8'h31};
            9'd120 : cmd_data = {16'h0D52, 8'h30};
            9'd121 : cmd_data = {16'h0D53, 8'h38};
            9'd122 : cmd_data = {16'h0D54, 8'h30};
            9'd123 : cmd_data = {16'h0D55, 8'h50};
            9'd124 : cmd_data = {16'h0D56, 8'h33};
            9'd125 : cmd_data = {16'h0D57, 8'h30};
            9'd126 : cmd_data = {16'h0D58, 8'h0A};
            9'd127 : cmd_data = {16'h0D59, 8'h20};
            9'd128 : cmd_data = {16'h0D5A, 8'h00};
            9'd129 : cmd_data = {16'h0D5B, 8'h00};
            9'd130 : cmd_data = {16'h0D5C, 8'h00};
            9'd131 : cmd_data = {16'h0D5D, 8'hFE};
            9'd132 : cmd_data = {16'h0D5E, 8'h00};
            9'd133 : cmd_data = {16'h0D5F, 8'h52};
            9'd134 : cmd_data = {16'h0D60, 8'h47};
            9'd135 : cmd_data = {16'h0D61, 8'h42};
            9'd136 : cmd_data = {16'h0D62, 8'h20};
            9'd137 : cmd_data = {16'h0D63, 8'h31};
            9'd138 : cmd_data = {16'h0D64, 8'h30};
            9'd139 : cmd_data = {16'h0D65, 8'h38};
            9'd140 : cmd_data = {16'h0D66, 8'h30};
            9'd141 : cmd_data = {16'h0D67, 8'h70};
            9'd142 : cmd_data = {16'h0D68, 8'h33};
            9'd143 : cmd_data = {16'h0D69, 8'h30};
            9'd144 : cmd_data = {16'h0D6A, 8'h0A};
            9'd145 : cmd_data = {16'h0D6B, 8'h20};
            9'd146 : cmd_data = {16'h0D6C, 8'h00};
            9'd147 : cmd_data = {16'h0D6D, 8'h00};
            9'd148 : cmd_data = {16'h0D6E, 8'h00};
            9'd149 : cmd_data = {16'h0D6F, 8'hFD};
            9'd150 : cmd_data = {16'h0D70, 8'h00};
            9'd151 : cmd_data = {16'h0D71, 8'h1D};
            9'd152 : cmd_data = {16'h0D72, 8'h3D};
            9'd153 : cmd_data = {16'h0D73, 8'h1F};
            9'd154 : cmd_data = {16'h0D74, 8'h22};
            9'd155 : cmd_data = {16'h0D75, 8'h08};
            9'd156 : cmd_data = {16'h0D76, 8'h00};
            9'd157 : cmd_data = {16'h0D77, 8'h0A};
            9'd158 : cmd_data = {16'h0D78, 8'h20};
            9'd159 : cmd_data = {16'h0D79, 8'h20};
            9'd160 : cmd_data = {16'h0D7A, 8'h20};
            9'd161 : cmd_data = {16'h0D7B, 8'h20};
            9'd162 : cmd_data = {16'h0D7C, 8'h20};
            9'd163 : cmd_data = {16'h0D7D, 8'h20};
            9'd164 : cmd_data = {16'h0D7E, 8'h01};
            9'd165 : cmd_data = {16'h0D7F, 8'h80};
            9'd166 : cmd_data = {16'h0D80, 8'h02};
            9'd167 : cmd_data = {16'h0D81, 8'h03};
            9'd168 : cmd_data = {16'h0D82, 8'h16};
            9'd169 : cmd_data = {16'h0D83, 8'h81};
            9'd170 : cmd_data = {16'h0D84, 8'h42};
            9'd171 : cmd_data = {16'h0D85, 8'hA2};
            9'd172 : cmd_data = {16'h0D86, 8'h01};
            9'd173 : cmd_data = {16'h0D87, 8'h67};
            9'd174 : cmd_data = {16'h0D88, 8'h03};
            9'd175 : cmd_data = {16'h0D89, 8'h0C};
            9'd176 : cmd_data = {16'h0D8A, 8'h00};
            9'd177 : cmd_data = {16'h0D8B, 8'h00};
            9'd178 : cmd_data = {16'h0D8C, 8'h00};
            9'd179 : cmd_data = {16'h0D8D, 8'h00};
            9'd180 : cmd_data = {16'h0D8E, 8'h0F};
            9'd181 : cmd_data = {16'h0D8F, 8'hE2};
            9'd182 : cmd_data = {16'h0D90, 8'h00};
            9'd183 : cmd_data = {16'h0D91, 8'h4A};
            9'd184 : cmd_data = {16'h0D92, 8'hE3};
            9'd185 : cmd_data = {16'h0D93, 8'h05};
            9'd186 : cmd_data = {16'h0D94, 8'h00};
            9'd187 : cmd_data = {16'h0D95, 8'h20};
            9'd188 : cmd_data = {16'h0D96, 8'h00};
            9'd189 : cmd_data = {16'h0D97, 8'h00};
            9'd190 : cmd_data = {16'h0D98, 8'h00};
            9'd191 : cmd_data = {16'h0D99, 8'h00};
            9'd192 : cmd_data = {16'h0D9A, 8'h00};
            9'd193 : cmd_data = {16'h0D9B, 8'h00};
            9'd194 : cmd_data = {16'h0D9C, 8'h00};
            9'd195 : cmd_data = {16'h0D9D, 8'h00};
            9'd196 : cmd_data = {16'h0D9E, 8'h00};
            9'd197 : cmd_data = {16'h0D9F, 8'h00};
            9'd198 : cmd_data = {16'h0DA0, 8'h00};
            9'd199 : cmd_data = {16'h0DA1, 8'h00};
            9'd200 : cmd_data = {16'h0DA2, 8'h00};
            9'd201 : cmd_data = {16'h0DA3, 8'h00};
            9'd202 : cmd_data = {16'h0DA4, 8'h00};
            9'd203 : cmd_data = {16'h0DA5, 8'h00};
            9'd204 : cmd_data = {16'h0DA6, 8'h00};
            9'd205 : cmd_data = {16'h0DA7, 8'h00};
            9'd206 : cmd_data = {16'h0DA8, 8'h00};
            9'd207 : cmd_data = {16'h0DA9, 8'h00};
            9'd208 : cmd_data = {16'h0DAA, 8'h00};
            9'd209 : cmd_data = {16'h0DAB, 8'h00};
            9'd210 : cmd_data = {16'h0DAC, 8'h00};
            9'd211 : cmd_data = {16'h0DAD, 8'h00};
            9'd212 : cmd_data = {16'h0DAE, 8'h00};
            9'd213 : cmd_data = {16'h0DAF, 8'h00};
            9'd214 : cmd_data = {16'h0DB0, 8'h00};
            9'd215 : cmd_data = {16'h0DB1, 8'h00};
            9'd216 : cmd_data = {16'h0DB2, 8'h00};
            9'd217 : cmd_data = {16'h0DB3, 8'h00};
            9'd218 : cmd_data = {16'h0DB4, 8'h00};
            9'd219 : cmd_data = {16'h0DB5, 8'h00};
            9'd220 : cmd_data = {16'h0DB6, 8'h00};
            9'd221 : cmd_data = {16'h0DB7, 8'h00};
            9'd222 : cmd_data = {16'h0DB8, 8'h00};
            9'd223 : cmd_data = {16'h0DB9, 8'h00};
            9'd224 : cmd_data = {16'h0DBA, 8'h00};
            9'd225 : cmd_data = {16'h0DBB, 8'h00};
            9'd226 : cmd_data = {16'h0DBC, 8'h00};
            9'd227 : cmd_data = {16'h0DBD, 8'h00};
            9'd228 : cmd_data = {16'h0DBE, 8'h00};
            9'd229 : cmd_data = {16'h0DBF, 8'h00};
            9'd230 : cmd_data = {16'h0DC0, 8'h00};
            9'd231 : cmd_data = {16'h0DC1, 8'h00};
            9'd232 : cmd_data = {16'h0DC2, 8'h00};
            9'd233 : cmd_data = {16'h0DC3, 8'h00};
            9'd234 : cmd_data = {16'h0DC4, 8'h00};
            9'd235 : cmd_data = {16'h0DC5, 8'h00};
            9'd236 : cmd_data = {16'h0DC6, 8'h00};
            9'd237 : cmd_data = {16'h0DC7, 8'h00};
            9'd238 : cmd_data = {16'h0DC8, 8'h00};
            9'd239 : cmd_data = {16'h0DC9, 8'h00};
            9'd240 : cmd_data = {16'h0DCA, 8'h00};
            9'd241 : cmd_data = {16'h0DCB, 8'h00};
            9'd242 : cmd_data = {16'h0DCC, 8'h00};
            9'd243 : cmd_data = {16'h0DCD, 8'h00};
            9'd244 : cmd_data = {16'h0DCE, 8'h00};
            9'd245 : cmd_data = {16'h0DCF, 8'h00};
            9'd246 : cmd_data = {16'h0DD0, 8'h00};
            9'd247 : cmd_data = {16'h0DD1, 8'h00};
            9'd248 : cmd_data = {16'h0DD2, 8'h00};
            9'd249 : cmd_data = {16'h0DD3, 8'h00};
            9'd250 : cmd_data = {16'h0DD4, 8'h00};
            9'd251 : cmd_data = {16'h0DD5, 8'h00};
            9'd252 : cmd_data = {16'h0DD6, 8'h00};
            9'd253 : cmd_data = {16'h0DD7, 8'h00};
            9'd254 : cmd_data = {16'h0DD8, 8'h00};
            9'd255 : cmd_data = {16'h0DD9, 8'h00};
            9'd256 : cmd_data = {16'h0DDA, 8'h00};
            9'd257 : cmd_data = {16'h0DDB, 8'h00};
            9'd258 : cmd_data = {16'h0DDC, 8'h00};
            9'd259 : cmd_data = {16'h0DDD, 8'h00};
            9'd260 : cmd_data = {16'h0DDE, 8'h00};
            9'd261 : cmd_data = {16'h0DDF, 8'h00};
            9'd262 : cmd_data = {16'h0DE0, 8'h00};
            9'd263 : cmd_data = {16'h0DE1, 8'h00};
            9'd264 : cmd_data = {16'h0DE2, 8'h00};
            9'd265 : cmd_data = {16'h0DE3, 8'h00};
            9'd266 : cmd_data = {16'h0DE4, 8'h00};
            9'd267 : cmd_data = {16'h0DE5, 8'h00};
            9'd268 : cmd_data = {16'h0DE6, 8'h00};
            9'd269 : cmd_data = {16'h0DE7, 8'h00};
            9'd270 : cmd_data = {16'h0DE8, 8'h00};
            9'd271 : cmd_data = {16'h0DE9, 8'h00};
            9'd272 : cmd_data = {16'h0DEA, 8'h00};
            9'd273 : cmd_data = {16'h0DEB, 8'h00};
            9'd274 : cmd_data = {16'h0DEC, 8'h00};
            9'd275 : cmd_data = {16'h0DED, 8'h00};
            9'd276 : cmd_data = {16'h0DEE, 8'h00};
            9'd277 : cmd_data = {16'h0DEF, 8'h00};
            9'd278 : cmd_data = {16'h0DF0, 8'h00};
            9'd279 : cmd_data = {16'h0DF1, 8'h00};
            9'd280 : cmd_data = {16'h0DF2, 8'h00};
            9'd281 : cmd_data = {16'h0DF3, 8'h00};
            9'd282 : cmd_data = {16'h0DF4, 8'h00};
            9'd283 : cmd_data = {16'h0DF5, 8'h00};
            9'd284 : cmd_data = {16'h0DF6, 8'h00};
            9'd285 : cmd_data = {16'h0DF7, 8'h00};
            9'd286 : cmd_data = {16'h0DF8, 8'h00};
            9'd287 : cmd_data = {16'h0DF9, 8'h00};
            9'd288 : cmd_data = {16'h0DFA, 8'h00};
            9'd289 : cmd_data = {16'h0DFB, 8'h00};
            9'd290 : cmd_data = {16'h0DFC, 8'h00};
            9'd291 : cmd_data = {16'h0DFD, 8'h00};
            9'd292 : cmd_data = {16'h0DFE, 8'h00};
            9'd293 : cmd_data = {16'h0DFF, 8'hC6};
            9'd294 : cmd_data = {16'h000C, 8'h00};
            9'd295 : cmd_data = {16'h000A, 8'h04};
            9'd296 : cmd_data = {16'h2000, 8'h0F};
            9'd297 : cmd_data = {16'h2001, 8'h00};
            9'd298 : cmd_data = {16'h2002, 8'h00};
            9'd299 : cmd_data = {16'h2003, 8'h01}; 
            
            9'd300 : cmd_data = {16'h209C, 8'h00};
            9'd301 : cmd_data = {16'h209D, 8'h00};
            9'd302 : cmd_data = {16'h209E, 8'h00};
            9'd303 : cmd_data = {16'h209F, 8'h00};
             
            9'd304 : cmd_data = {16'h1010, 8'h01};
            9'd305 : cmd_data = {16'h1024, 8'h00};
            9'd306 : cmd_data = {16'h0200, 8'h00};
            9'd307 : cmd_data = {16'h1024, 8'h70};
            9'd308 : cmd_data = {16'h0200, 8'h07};
            9'd309 : cmd_data = {16'h0215, 8'h01};
            9'd310 : cmd_data = {16'h0215, 8'h00};
            default: cmd_data = 24'h000000;
       endcase 
    end
endfunction
localparam [4:0] PROBE_WR=0, PROBE_RD=1, INIT=2, STATUS_RD=3,
    SETTING=4, PHASE_RD=5, POLL_WAIT=6, PREPARE=7, LAUNCH=8,
    WAIT_BUSY=9, WAIT_DONE=10, COMPLETE=11, RETRY_WAIT=12, RESTART_WAIT=13,
    SETTLE=14, PHASE_RMW=15, PHASE_WRITE=16, CONFIG_WAIT=17;
reg [4:0] state, operation;
reg [8:0] index;
reg [2:0] retry_count;
reg [31:0] timer;
reg [7:0] previous_status;
reg [23:0] status_shadow;
reg [1:0] candidate_status, status_samples, recovery_attempts;
reg status_initialized, recovery_escalated, readback_valid;
reg [7:0] full_init_count, setting_count;
reg [3:0] recovery_reason;
reg [31:0] recovery_cooldown;
reg [63:0] config_readback, readback_shadow;
reg reconfigure_pending;
reg [23:0] command;
reg auto_recovery, phase_override, desired_phase, control_busy, control_error, slow_config;
reg [7:0] phase_write_value;
reg [31:0] active_request, control_ack;
wire new_control = control_request[31:24] == 8'h72 && control_request != control_ack;
wire [31:0] status_sample = {data_out,status_shadow};
wire status_confirmed = status_samples != 0 && candidate_status == status_shadow[17:16];
assign debug_state = state;
assign config_diag = {config_readback,
    8'd1,full_init_count,setting_count,recovery_reason,
    recovery_escalated,status_initialized,readback_valid,init_over,32'h52584331};
assign control_diag = {freq_rec,
    8'd2,8'd0,phase_readback,1'd0,slow_config,sample_rising,desired_phase,phase_override,
    control_error,control_busy,auto_recovery,control_ack,32'h52584332};

always @(*) begin
    command = 24'h00035a;
    case (operation)
        INIT: command = cmd_data(index);
        STATUS_RD: command = cmd_data(9'd300 + index);
        SETTING: command = cmd_data(9'd304 + index);
        PHASE_RD: case(index)
            0: command = 24'h000a00;
            1: command = 24'h100000;
            2: command = 24'h102d00;
            3: command = 24'h101000;
            4: command = 24'h102400;
            5: command = 24'h020000;
            6: command = 24'h021500;
            default: command = 24'h128100;
        endcase
        PHASE_RMW: command = 24'h128100;
        PHASE_WRITE: command = {16'h1281,phase_write_value};
        default: command = 24'h00035a;
    endcase
end

always @(posedge clk or negedge rstn) begin
    if (!rstn) begin
        state <= PREPARE; operation <= PROBE_WR; index <= 0;
        init_over <= 0; iic_trig <= 0; w_r <= 1;
        addr <= 0; data_in <= 0; retry_count <= 0; timer <= 0;
        iic_abort <= 0; error_seen <= 0; error_count <= 0;
        last_error_addr <= 0; freq_rec <= 0; previous_status <= 0;
        reconfigure_pending <= 0; phase_readback <= 0;
        status_shadow <= 0; candidate_status <= 0; status_samples <= 0;
        status_initialized <= 0; recovery_attempts <= 0; recovery_escalated <= 0;
        full_init_count <= 0; setting_count <= 0; recovery_reason <= 1;
        config_readback <= 0; readback_shadow <= 0; readback_valid <= 0;
        recovery_cooldown <= 0;
        auto_recovery <= 1; phase_override <= 0; desired_phase <= 0;
        slow_config <= 0;
        sample_rising <= 0; phase_write_value <= 0;
        active_request <= 0; control_ack <= 0; control_busy <= 0; control_error <= 0;
    end else begin
        iic_trig <= 0;
        iic_abort <= 0;
        if (reconfigure) reconfigure_pending <= 1;
        if (recovery_cooldown != 0) recovery_cooldown <= recovery_cooldown - 1'b1;
        if (video_valid === 1'b1) begin
            recovery_attempts <= 0; recovery_escalated <= 0;
            recovery_cooldown <= 0; reconfigure_pending <= 0;
        end
        case (state)
            PREPARE: begin
                addr <= command[23:8]; data_in <= command[7:0];
                w_r <= (operation == PROBE_WR || operation == INIT || operation == SETTING || operation == PHASE_WRITE);
                timer <= 0; state <= LAUNCH;
            end
            LAUNCH: if (!busy) begin
                iic_trig <= 1; timer <= 0; state <= WAIT_BUSY;
            end else if (timer == TX_TIMEOUT_CYCLES-1) begin
                state <= RESTART_WAIT; timer <= 0; init_over <= 0; iic_abort <= 1;
                error_seen <= 1; last_error_addr <= addr;
                if (error_count != 16'hffff) error_count <= error_count + 1'b1;
            end else timer <= timer + 1'b1;
            WAIT_BUSY, WAIT_DONE: begin
                if (state == WAIT_BUSY && busy) state <= WAIT_DONE;
                if (state == WAIT_DONE && !busy) begin
                    timer <= 0;
                    if (txn_error || (operation == PROBE_RD && data_out != 8'h5a)) begin
                        error_seen <= 1; last_error_addr <= addr; init_over <= 0;
                        if (error_count != 16'hffff) error_count <= error_count + 1'b1;
                        if (retry_count == MAX_RETRIES-1) begin
                            retry_count <= 0; state <= RESTART_WAIT;
                        end else begin
                            retry_count <= retry_count + 1'b1; state <= RETRY_WAIT;
                        end
                    end else begin
                        retry_count <= 0; state <= COMPLETE;
                    end
                end else if (timer == TX_TIMEOUT_CYCLES-1) begin
                    timer <= 0; init_over <= 0; iic_abort <= 1; state <= RESTART_WAIT;
                    error_seen <= 1; last_error_addr <= addr;
                    if (error_count != 16'hffff) error_count <= error_count + 1'b1;
                end else timer <= timer + 1'b1;
            end
            COMPLETE: begin
                timer <= 0;
                case (operation)
                    PROBE_WR: begin operation <= PROBE_RD; state <= PREPARE; end
                    PROBE_RD: begin operation <= INIT; index <= 0; state <= PREPARE; end
                    INIT: begin
                        if (index == 299) begin
                            operation <= STATUS_RD; index <= 0; status_initialized <= 0;
                            status_samples <= 0; readback_valid <= 0;
                        end
                        else index <= index + 1'b1;
                        // Keep EDID RAM streaming fast; pace only the vendor
                        // analog/output setup writes when explicitly requested.
                        state <= slow_config && (index<30 || index>=294) ? CONFIG_WAIT : PREPARE;
                    end
                    STATUS_RD: begin
                        case (index)
                            0: status_shadow[7:0] <= data_out;
                            1: status_shadow[15:8] <= data_out;
                            2: status_shadow[23:16] <= data_out;
                        endcase
                        if (index == 3) begin
                            index <= 0;
                            // Publish one complete four-byte poll. Debounce all
                            // transitions of the vendor status field, including 2 -> 3.
                            freq_rec <= status_sample;
                            candidate_status <= status_shadow[17:16];
                            status_samples <= status_confirmed ? 2 : 1;
                            if (auto_recovery && reconfigure_pending && video_valid !== 1'b1 &&
                                recovery_cooldown == 0 && recovery_attempts == 2 &&
                                !recovery_escalated) begin
                                operation <= INIT; init_over <= 0; readback_valid <= 0;
                                reconfigure_pending <= 0; recovery_escalated <= 1;
                                recovery_reason <= 4; state <= PREPARE;
                                if (full_init_count != 255) full_init_count <= full_init_count + 1'b1;
                                recovery_cooldown <= RECOVERY_COOLDOWN_CYCLES;
                            end else if (!init_over ||
                                (auto_recovery && reconfigure_pending && video_valid !== 1'b1 && recovery_cooldown == 0) ||
                                (auto_recovery && status_initialized && status_confirmed &&
                                 previous_status[1:0] != status_shadow[17:16])) begin
                                operation <= SETTING; init_over <= 0;
                                readback_valid <= 0; reconfigure_pending <= 0; state <= PREPARE;
                                previous_status <= status_shadow[23:16]; status_initialized <= 1;
                                if (setting_count != 255) setting_count <= setting_count + 1'b1;
                                if (reconfigure_pending && video_valid !== 1'b1 && recovery_cooldown == 0) begin
                                    recovery_reason <= 3;
                                    if (recovery_attempts != 2) recovery_attempts <= recovery_attempts + 1'b1;
                                    if (recovery_escalated) recovery_cooldown <= RECOVERY_COOLDOWN_CYCLES;
                                end else if (init_over) recovery_reason <= 2;
                            end else begin operation <= PHASE_RD; state <= PREPARE; end
                        end else begin index <= index + 1'b1; state <= PREPARE; end
                    end
                    SETTING: begin
                        if (index == 6) begin
                            index <= 0;
                            operation <= phase_override ? PHASE_RMW : PHASE_RD;
                            state <= phase_override ? (slow_config ? CONFIG_WAIT : PREPARE) : SETTLE;
                        end else begin
                            index <= index + 1'b1;
                            state <= slow_config ? CONFIG_WAIT : PREPARE;
                        end
                    end
                    PHASE_RMW: begin
                        // Only the documented Pclk_inv bit is changed.
                        phase_write_value <= (data_out & 8'hfb) | {5'd0,desired_phase,2'd0};
                        operation <= PHASE_WRITE; state <= PREPARE;
                    end
                    PHASE_WRITE: begin index <= 0; operation <= PHASE_RD; state <= SETTLE; end
                    PHASE_RD: begin
                        readback_shadow[index*8 +: 8] <= data_out;
                        if (index == 7) begin
                            phase_readback <= data_out;
                            config_readback <= {data_out,readback_shadow[55:0]};
                            readback_valid <= 1; index <= 0; state <= POLL_WAIT;
                            if (control_busy) begin
                                control_ack <= active_request; control_busy <= 0;
                                if (phase_override && data_out[2] != desired_phase) control_error <= 1;
                            end
                        end else begin index <= index + 1'b1; state <= PREPARE; end
                    end
                    default: state <= RESTART_WAIT;
                endcase
            end
            POLL_WAIT: begin
                if (new_control && !control_busy) begin
                    timer <= 0; index <= 0; active_request <= control_request;
                    control_error <= 0;
                    case(control_request[15:8])
                        1: begin
                            desired_phase <= control_request[0]; phase_override <= 1;
                            control_busy <= 1; init_over <= 0; readback_valid <= 0;
                            operation <= PHASE_RMW; state <= PREPARE;
                        end
                        2: begin
                            control_busy <= 1; init_over <= 0; readback_valid <= 0;
                            operation <= SETTING; state <= PREPARE;
                        end
                        3: begin
                            control_busy <= 1; init_over <= 0; readback_valid <= 0;
                            operation <= INIT; state <= PREPARE; reconfigure_pending <= 0;
                        end
                        4: begin
                            auto_recovery <= control_request[0]; reconfigure_pending <= 0;
                            control_ack <= control_request;
                        end
                        5: begin
                            sample_rising <= control_request[0]; control_busy <= 1;
                            init_over <= 0; readback_valid <= 0;
                            operation <= PHASE_RD; state <= SETTLE;
                        end
                        6: begin slow_config <= control_request[0]; control_ack <= control_request; end
                        default: begin control_error <= 1; control_ack <= control_request; end
                    endcase
                end else if (timer == POLL_CYCLES-1 || (auto_recovery && reconfigure_pending && recovery_cooldown == 0)) begin
                    timer <= 0; operation <= STATUS_RD; index <= 0; state <= PREPARE;
                end else timer <= timer + 1'b1;
            end
            SETTLE: begin
                if (timer == SETTLE_CYCLES-1) begin
                    timer <= 0; init_over <= 1; state <= PREPARE;
                end else timer <= timer + 1'b1;
            end
            CONFIG_WAIT: begin
                if (timer == CONFIG_STEP_CYCLES-1) begin timer <= 0; state <= PREPARE; end
                else timer <= timer + 1'b1;
            end
            RETRY_WAIT: begin
                if (timer == RETRY_CYCLES-1) begin timer <= 0; state <= PREPARE; end
                else timer <= timer + 1'b1;
            end
            RESTART_WAIT: begin
                init_over <= 0; readback_valid <= 0; recovery_reason <= 5;
                if (timer == POLL_CYCLES-1) begin
                    timer <= 0; retry_count <= 0; operation <= PROBE_WR;
                    index <= 0; reconfigure_pending <= 0; state <= PREPARE;
                end else timer <= timer + 1'b1;
            end
            default: begin state <= RESTART_WAIT; timer <= 0; init_over <= 0; end
        endcase
    end
end
endmodule
