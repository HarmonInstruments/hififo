module pcie_from_pc_fifo
  (
   input 	     clock,
   input 	     reset,
   output reg [1:0]  interrupt = 0,
   output [63:0]     status, 
   // PIO
   input 	     pio_wvalid,
   input [63:0]      pio_wdata,
   input [12:0]      pio_addr, 
   // read completion
   input 	     rc_valid,
   input [7:0] 	     rc_tag,
   input [5:0] 	     rc_index,
   input [63:0]      rc_data,
   // read request
   output 	     rr_valid,
   output [63:0]     rr_addr,
   output [7:0]      rr_tag,
   input 	     rr_ready,
   // FIFO
   input 	     fifo_clock, // for all FIFO signals
   input 	     fifo_read,
   output reg [63:0] fifo_read_data,
   output reg 	     fifo_read_valid = 0
   );

   // clock
   reg [42:0] 	     pt [31:0];
   reg [42:0] 	     pt_q;
   reg [63:0] 	     fifo_bram [511:0];
   reg [63:0] 	     fifo_bram_oreg;
   reg [7:0] 	     block_filled = 0;
   reg [2:0] 	     p_read = 0;
   reg [16:0] 	     p_write = 0;
   reg [16:0] 	     p_request = 0;
   reg [16:0] 	     p_stop = 0;
   reg [16:0] 	     p_int = 0;
   wire  	     write = (rc_tag[7:3] == 0) && rc_valid;
   wire [8:0] 	     write_address = {rc_tag[2:0],rc_index};
   wire 	     write_last = write && (rc_index == 6'h3F);
   wire [2:0] 	     prp2 = p_request[2:0] + 2'd2;
   assign rr_valid = ~rr_ready & (prp2 != p_read[2:0]) & (p_request != p_stop);
   assign rr_addr = {pt_q, p_request[13:0], 9'd0};
   assign rr_tag = p_request[2:0];
   assign status = {p_write, 9'd0};
   wire 	     p_read_6_clk, p_read_inc128;
   
   always @ (posedge clock)
     begin
	pt_q <= pt[p_request[16:12]];
	if(pio_wvalid && (pio_addr[12:5] == 2))
	  pt[pio_addr[4:0]] <= pio_wdata[63:21];
	if(write)
	  fifo_bram[write_address] <= rc_data;
	if(reset)
	  block_filled <= 8'd0;
	else
	  begin
	     block_filled[0] <= (write_last && (rc_tag == 0)) || ((p_write == 0) ? 1'b0 : block_filled[0]);
	     block_filled[1] <= (write_last && (rc_tag == 1)) || ((p_write == 1) ? 1'b0 : block_filled[1]);
	     block_filled[2] <= (write_last && (rc_tag == 2)) || ((p_write == 2) ? 1'b0 : block_filled[2]);
	     block_filled[3] <= (write_last && (rc_tag == 3)) || ((p_write == 3) ? 1'b0 : block_filled[3]);
	     block_filled[4] <= (write_last && (rc_tag == 4)) || ((p_write == 4) ? 1'b0 : block_filled[4]);
	     block_filled[5] <= (write_last && (rc_tag == 5)) || ((p_write == 5) ? 1'b0 : block_filled[5]);
	     block_filled[6] <= (write_last && (rc_tag == 6)) || ((p_write == 6) ? 1'b0 : block_filled[6]);
	     block_filled[7] <= (write_last && (rc_tag == 7)) || ((p_write == 7) ? 1'b0 : block_filled[7]);
	  end
	p_stop <= reset ? 1'b0 : (pio_wvalid && (pio_addr == 6) ? pio_wdata[25:9] : p_stop);
	p_int <= reset ? 1'b0 : (pio_wvalid && (pio_addr == 7) ? pio_wdata[25:9] : p_int);
	p_read <= reset ? 1'b0 : p_read + p_read_inc128;
	p_write <= reset ? 1'b0 : p_write + block_filled[p_write[2:0]];
	p_request <= reset ? 1'b0 : p_request + rr_ready;
	interrupt <= {(p_stop == p_write), (p_int == p_write)};
     end
   
   oneshot_dualedge oneshot0(.clock(clock), .in(p_read_6_clk), .out(p_read_inc128));

   // fifo_clock
   reg [8:0] 	  p_read_fclk = 0;
   wire [2:0] 	  p_write_fclk;
   wire 	  reset_fclk;
   gray_sync_3 sync0(.clock_in(clock), .in(p_write[2:0]), .clock_out(fifo_clock), .out(p_write_fclk));
   sync sync1(.clock(clock), .in(p_read_fclk[6]), .out(p_read_6_clk));
   sync sync_reset(.clock(fifo_clock), .in(reset), .out(reset_fclk));
   
   reg 		  read_valid_q = 0;
   wire 	  read_valid = fifo_read && (p_read_fclk != {p_write_fclk,6'd0});
   
   always @ (posedge fifo_clock)
     begin
	fifo_bram_oreg <= fifo_bram[p_read_fclk];
	fifo_read_data <= fifo_bram_oreg;
	if(reset_fclk)
	  begin
	     fifo_read_valid <= 1'b0;
	     p_read_fclk <= 1'b0;
	  end
	else
	  begin
	     p_read_fclk <= p_read_fclk + read_valid;
	     read_valid_q <= read_valid;
	     fifo_read_valid <= read_valid_q;
	  end
     end
endmodule


