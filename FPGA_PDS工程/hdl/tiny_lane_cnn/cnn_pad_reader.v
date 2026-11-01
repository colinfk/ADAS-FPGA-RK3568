// One-cycle synchronous source RAM; feed explicit zero halo into line buffers.
module cnn_pad_reader #(parameter WIDTH=32, HEIGHT=64, DATA_BITS=8, ADDR_BITS=11)(
 input clk,rst_n,start,
 output reg [ADDR_BITS-1:0] rd_addr,input [DATA_BITS-1:0] rd_data,
 output out_valid,input out_ready,output reg [DATA_BITS-1:0] out_data,
 output reg done
);
localparam IDLE=0,READ=1,LATCH=2,SEND=3;
reg [1:0] state;
reg [7:0] x,y;
wire border=(x==0 || y==0 || x==WIDTH+1 || y==HEIGHT+1);
assign out_valid=state==SEND;
always @* begin
 rd_addr=0;
 if(!border) rd_addr=(y-1)*WIDTH+(x-1);
end
always @(posedge clk) begin
 if(!rst_n || start) begin
  state<=start ? READ : IDLE;x<=0;y<=0;out_data<=0;done<=0;
 end else case(state)
  READ: state<=LATCH;
  LATCH: begin out_data<=border ? 0 : rd_data;state<=SEND;end
  SEND: if(out_ready) begin
   if(x==WIDTH+1 && y==HEIGHT+1) begin state<=IDLE;done<=1;end
   else begin
    if(x==WIDTH+1) begin x<=0;y<=y+1'b1;end else x<=x+1'b1;
    state<=READ;
   end
  end
 endcase
end
endmodule

