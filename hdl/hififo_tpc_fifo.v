module hififo_tpc_fifo
  (
   input 	     clock,
   input 	     reset,
   input [2:0] 	     channel,
   // request
   output 	     ready_for_read,
   // RAM interface
   input 	     read,
   input [8:0] 	     read_address,
   output reg [63:0] read_data,
   // FIFO
   input 	     fifo_clock,
   input 	     fifo_write,
   input [63:0]      fifo_data,
   output reg 	     fifo_ready = 0
   );

   // clock
   reg [63:0] 	     fifo [511:0];
   wire [8:0] 	     p_in;
   reg [8:0] 	     p_out = 0;
   wire [8:0] 	     fifo_count = p_in - p_out;

   assign ready_for_read = fifo_count < 15;
   assign p_in[5:0] = 0;
   
   always @ (posedge clock)
     begin
	read_data <= fifo[read_address];
	if(reset)
	  p_out <= 1'b0;
	else if(read)
	  p_out <= p_out + 1'b1;
     end

   // fifo_clock
   reg [8:0] 	  p_in_fclk = 0;
   wire [8:0] 	  p_out_fclk;
   wire 	  reset_fclk;
   wire [8:0] 	  fifo_count_fclk = p_in_fclk - p_out_fclk;
   
   gray_sync_3 sync0(.clock_in(clock), .in(p_out[8:6]), .clock_out(fifo_clock), .out(p_out_fclk[8:6]));
   gray_sync_3 sync1(.clock_in(fifo_clock), .in(p_in_fclk[8:6]), .clock_out(clock), .out(p_in[8:6]));
   sync sync_reset(.clock(clock), .in(reset), .out(reset_fclk));
   assign p_out_fclk[5:0] = 6'd0;
   
   always @ (posedge fifo_clock)
     begin
	p_in_fclk <= reset_fclk ? 1'b0 : p_in_fclk + fifo_write;
	fifo_ready <= fifo_count_fclk < 496;
	if(fifo_write && (fifo_count_fclk != 511))
	  fifo[p_in_fclk] <= fifo_data;
     end
endmodule
