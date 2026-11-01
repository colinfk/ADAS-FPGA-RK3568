// Four-phase frame command across PCIe/DDR domains. PCIe keeps run asserted
// for one grant; DDR starts once, then waits for run to fall. Idle is returned
// only after cancellation has removed a pending request and drained DDR reads.
// Either endpoint reset terminates both command synchronizers, without
// resetting the DDR reader while a native burst is outstanding.
module video_frame_bridge (
    input pcie_clk, pcie_rst_n, input i_run,
    input ddr_clk, ddr_rst_n,
    input i_reader_busy, i_reader_pending,
    output reg o_reader_start,
    output o_reader_cancel,
    output o_source_idle,
    output o_fifo_reset_n
);
wire link_rst_n = pcie_rst_n && ddr_rst_n;
reg [1:0] pcie_reset_pipe, ddr_reset_pipe;
reg run_sync1, run_sync2;
reg idle_sync1, idle_sync2;
reg granted;
reg pcie_armed;
wire accepted_run=i_run && pcie_armed;
always @(posedge pcie_clk or negedge link_rst_n)
    if (!link_rst_n) pcie_reset_pipe<=0;
    else pcie_reset_pipe<={pcie_reset_pipe[0],1'b1};
always @(posedge ddr_clk or negedge link_rst_n)
    if (!link_rst_n) ddr_reset_pipe<=0;
    else ddr_reset_pipe<={ddr_reset_pipe[0],1'b1};
wire ddr_local_rst_n=ddr_reset_pipe[1];
wire pcie_local_rst_n=pcie_reset_pipe[1];
assign o_reader_cancel=!ddr_local_rst_n || !run_sync2;
wire ddr_idle=ddr_local_rst_n && !run_sync2 && !i_reader_busy &&
              !i_reader_pending && !o_reader_start;
assign o_source_idle=pcie_local_rst_n && idle_sync2 && !i_run;
// This shared asynchronous assertion reaches both FIFO endpoints. Each
// endpoint synchronizes deassertion locally inside the FIFO wrapper.
assign o_fifo_reset_n=link_rst_n && accepted_run;
always @(posedge pcie_clk or negedge pcie_local_rst_n) begin
    if(!pcie_local_rst_n) pcie_armed<=0;
    else if(!i_run) pcie_armed<=1;
end
always @(posedge ddr_clk or negedge ddr_local_rst_n) begin
    if(!ddr_local_rst_n) begin
        run_sync1<=0;run_sync2<=0;granted<=0;o_reader_start<=0;
    end else begin
        run_sync1<=accepted_run;run_sync2<=run_sync1;o_reader_start<=0;
        if(!run_sync2) granted<=0;
        else if(!granted && !i_reader_busy && !i_reader_pending) begin
            granted<=1;o_reader_start<=1;
        end
    end
end
always @(posedge pcie_clk or negedge pcie_local_rst_n) begin
    if(!pcie_local_rst_n) begin idle_sync1<=0;idle_sync2<=0;end
    else begin idle_sync1<=ddr_idle;idle_sync2<=idle_sync1;end
end
endmodule
