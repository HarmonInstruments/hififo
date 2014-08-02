module hififo_tpc_fifo
  (
   input 	     clock,
   input 	     reset,
   output reg [1:0]  interrupt = 0,
   output [63:0]     status,
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
   output reg 	     fifo_ready = 0
   );
   
   reg [63:0] 	     wrr[0:511];
   reg [63:0] 	     wrr_q0, wrr_q1;
   
   // clock
   reg [42:0] 	     pt [31:0];
   reg [42:0] 	     pt_q;
   reg [4:0] 	     p_in; // 128 bytes
   reg [22:0] 	     p_out; // 8 bytes
   reg [18:0] 	     p_stop; // 128 bytes
   reg [18:0] 	     p_int; // 128 bytes
   wire [4:0] 	     fifo_count = p_in[4:0] - p_out[8:4];
   wire 	     in_inc128;
   reg 		     wr_valid_prev;
         
   assign wr_addr = {pt_q, p_out[17:4], 7'd0};
   assign status = {p_out[22:4], 7'd0};
   
   always @ (posedge clock)
     begin
	pt_q <= pt[p_out[22:18]];
	if(reset | wr_ready)
	  wr_valid <= 1'b0;
	else if((p_out[3:0] == 0) && (fifo_count != 0) && (p_out[22:4] != p_stop))
	  wr_valid <= 1'b1;
	if(wr_ready | wr_valid)
	  wr_data <= wrr_q0;
	if(wr_ready | (wr_valid & ~wr_valid_prev))
	  wrr_q0 <= wrr[p_out[8:0] + wr_ready];
	wr_valid_prev <= wr_valid;
	
	p_out <= reset ? 1'b0 : p_out + wr_ready;
	p_in <= reset ? 1'b0 : p_in + in_inc128;
	p_stop <= reset ? 1'b0 : ((pio_wvalid && (pio_addr == 3)) ? pio_wdata[26:7] : p_stop);
	p_int  <= reset ? 1'b0 : ((pio_wvalid && (pio_addr == 4)) ? pio_wdata[26:7] : p_int );
	if(pio_wvalid && (pio_addr[12:5] == 1))
	  pt[pio_addr[4:0]] <= pio_wdata[63:21];
	interrupt <= {(p_stop == p_out[22:4]), (p_int == p_out[22:4])};
     end

   wire 	     p_in_4_clk;
   oneshot_dualedge oneshot0(.clock(clock), .in(p_in_4_clk), .out(in_inc128));

   // fifo_clock
   reg [8:0] 	     wrr_addr;
   wire [2:0] 	     p_out_fclk;
   wire 	     reset_fclk;
   wire [2:0] 	     fifo_count_fclk = wrr_addr[8:6] - p_out_fclk;
      
   gray_sync_3 sync0(.clock_in(clock), .in(p_out[8:6]), .clock_out(fifo_clock), .out(p_out_fclk));
   sync sync1(.clock(clock), .in(wrr_addr[4]), .out(p_in_4_clk));
   sync sync2(.clock(fifo_clock), .in(reset), .out(reset_fclk));
   always @ (posedge fifo_clock)
     begin
	if(fifo_write && (fifo_count_fclk < 7))
	  wrr[wrr_addr] <= fifo_data;
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

