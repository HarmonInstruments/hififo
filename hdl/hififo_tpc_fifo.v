module hififo_tpc_fifo
  (
   input 	     clock,
   input 	     reset,
   output reg [1:0]  interrupt = 0,
   output [31:0]     status,
   // PIO
   input 	     pio_wvalid,
   input [63:0]      pio_wdata,
   input [12:0]      pio_addr, 
   // request
   output reg 	     wr_valid = 0,
   output [63:0]     wr_addr,
   input 	     wr_ready,
   output reg [63:0] wr_data,
   // FIFO
   input 	     fifo_clock,
   input 	     fifo_write,
   input [63:0]      fifo_data,
   output 	     fifo_ready
   );
   
   // clock
   reg [42:0] 	     pt [31:0];
   reg [42:0] 	     pt_q;
   reg [22:0] 	     p_out; // 8 bytes
   reg [18:0] 	     p_stop; // 128 bytes
   reg [18:0] 	     p_int; // 128 bytes
   wire 	     o_almost_empty;
   wire [63:0] 	     wr_data_i;
   
   assign wr_addr = {pt_q, p_out[17:4], 7'd0};
   assign status = {p_out[22:4], 7'd0};
   
   always @ (posedge clock)
     begin
	pt_q <= pt[p_out[22:18]];
	if(reset | wr_ready)
	  wr_valid <= 1'b0;
	else if((p_out[3:0] == 0) && ~o_almost_empty && (p_out[22:4] != p_stop))
	  wr_valid <= 1'b1;
	wr_data <= wr_data_i;
	p_out <= reset ? 1'b0 : p_out + wr_ready;
	p_stop <= reset ? 1'b0 : ((pio_wvalid && (pio_addr == 3)) ? pio_wdata[26:7] : p_stop);
	p_int  <= reset ? 1'b0 : ((pio_wvalid && (pio_addr == 4)) ? pio_wdata[26:7] : p_int );
	if(pio_wvalid && (pio_addr[12:5] == 1))
	  pt[pio_addr[4:0]] <= pio_wdata[63:21];
	interrupt <= {(p_stop == p_out[22:4]), (p_int == p_out[22:4])};
     end

   fwft_fifo #(.NBITS(64)) tpc_fifo
     (
      .reset(reset),
      .i_clock(fifo_clock),
      .i_data(fifo_data),
      .i_valid(fifo_write),
      .i_ready(fifo_ready),
      .o_clock(clock),
      .o_read(wr_ready),
      .o_data(wr_data_i),
      .o_valid(),
      .o_almost_empty(o_almost_empty)
      );
   
endmodule