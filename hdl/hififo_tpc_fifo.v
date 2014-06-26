module hififo_tpc_fifo
  (
   input 	    clock,
   input 	    reset,
   output reg 	    interrupt = 0,
   output [63:0]    status,
   // PIO
   input 	    pio_wvalid,
   input [63:0]     pio_wdata,
   input [12:0]     pio_addr, 
   // write request ram - fifo clock domain
   output 	    wrr_write,
   output reg [8:0] wrr_addr,
   output [63:0]    wrr_data,
   // request
   output reg 	    wr_valid = 0,
   output [63:0]    wr_addr,
   input 	    wr_ready,
   // FIFO
   input 	    fifo_clock,
   input 	    fifo_write,
   input [63:0]     fifo_data,
   output reg 	    fifo_ready = 0
   );

   assign wrr_data = fifo_data;
   
   // clock
   reg [42:0] 	     pt [31:0];
   reg [42:0] 	     pt_q;
   reg [18:0] 	     p_in; // 128 bytes
   reg [18:0] 	     p_out; // 128 bytes
   reg [18:0] 	     p_stop; // 128 bytes
   reg [18:0] 	     p_int; // 128 bytes
   wire [4:0] 	     fifo_count = p_in[4:0] - p_out[4:0];
   wire 	     in_inc128;
   
   assign wr_addr = {pt_q, p_out[13:0], 7'd0};
   assign status = {p_out, 7'd0};
   
   always @ (posedge clock)
     begin
	p_out <= reset ? 1'b0 : p_out + wr_ready;
	pt_q <= pt[p_out[18:14]];
	if(reset)
	  wr_valid <= 1'b0;
	else if(wr_ready)
	  wr_valid <= 1'b0;
	else if((fifo_count != 0) && (p_out != p_stop))
	  wr_valid <= 1'b1;
	if(reset)
	  p_out <= 1'b0;
	else if(wr_ready)
	  p_out <= p_out + 1'b1;
	if(reset)
	  p_in <= 1'b0;
	else if(1)
	  p_in <= p_in + in_inc128;
	if(reset)
	  p_stop <= 1'b0;
	else if(pio_wvalid && (pio_addr == 10))
	  p_stop <= pio_wdata[26:7];
	if(pio_wvalid && (pio_addr[12:5] == 1))
	  pt[pio_addr[4:0]] <= pio_wdata[63:21];
     end

   wire 	     p_in_4_clk;
   oneshot_dualedge oneshot0(.clock(clock), .in(p_in_4_clk), .out(in_inc128));

   // fifo_clock
   wire [2:0] 	     p_out_fclk;
   wire 	  reset_fclk;
   wire [2:0] 	  fifo_count_fclk = wrr_addr[8:6] - p_out_fclk;
      
   gray_sync_3 sync0(.clock_in(clock), .in(p_out[4:2]), .clock_out(fifo_clock), .out(p_out_fclk));
   sync sync1(.clock(clock), .in(wrr_addr[4]), .out(p_in_4_clk));
   sync sync2(.clock(fifo_clock), .in(reset), .out(reset_fclk));
   assign wrr_write = fifo_write && (fifo_count_fclk < 7);
   always @ (posedge fifo_clock)
     begin
	wrr_addr <= reset_fclk ? 1'b0 : wrr_addr + fifo_write;
	fifo_ready <= (fifo_count_fclk < 6) && ~reset_fclk;
     end
endmodule

module oneshot_dualedge(input clock, input in, output reg out=0);
   reg in_prev = 0;
   always @ (posedge clock)
     begin
	in_prev <= in;
	out <= in^in_prev;
     end
endmodule

