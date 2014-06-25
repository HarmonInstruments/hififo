module pcie_from_pc_fifo
  (
   input 	     clock,
   input 	     reset,
   input [2:0] 	     channel,
   // read completion
   input 	     completion_valid,
   input [7:0] 	     completion_tag,
   input [5:0] 	     completion_index,
   input [63:0]      completion_data,
   // read request
   output 	     read_request_valid,
   input 	     read_request_ready,
   // FIFO
   input 	     fifo_clock, // for all FIFO signals
   input 	     fifo_read,
   output reg [63:0] fifo_read_data = 0,
   output reg 	     fifo_read_valid = 0
   );

   // clock
   reg [63:0] 	     fifo_bram [511:0];
   reg [7:0] 	     block_filled = 0;
   wire [2:0] 	     p_read;
   reg [2:0] 	     p_write = 0;
   reg [2:0] 	     p_request = 0;
   wire  	     write = (completion_tag[7:3] == channel) && completion_valid;
   wire [8:0] 	     write_address = {completion_tag[2:0],completion_index};
   wire 	     write_last = write && (completion_index == 6'h3F);
   wire [2:0] 	     prp2 = p_request + 2'd2;
   assign read_request_valid = ~read_request_ready & (prp2 != p_read);
   
   always @ (posedge clock)
     begin
	if(write)
	  fifo_bram[write_address] <= completion_data;
	if(reset)
	  block_filled <= 8'd0;
	else
	  begin
	     block_filled[0] <= (write_last && (completion_tag == 0)) || ((p_write == 0) ? 1'b0 : block_filled[0]);
	     block_filled[1] <= (write_last && (completion_tag == 1)) || ((p_write == 1) ? 1'b0 : block_filled[1]);
	     block_filled[2] <= (write_last && (completion_tag == 2)) || ((p_write == 2) ? 1'b0 : block_filled[2]);
	     block_filled[3] <= (write_last && (completion_tag == 3)) || ((p_write == 3) ? 1'b0 : block_filled[3]);
	     block_filled[4] <= (write_last && (completion_tag == 4)) || ((p_write == 4) ? 1'b0 : block_filled[4]);
	     block_filled[5] <= (write_last && (completion_tag == 5)) || ((p_write == 5) ? 1'b0 : block_filled[5]);
	     block_filled[6] <= (write_last && (completion_tag == 6)) || ((p_write == 6) ? 1'b0 : block_filled[6]);
	     block_filled[7] <= (write_last && (completion_tag == 7)) || ((p_write == 7) ? 1'b0 : block_filled[7]);
	  end
	p_write <= reset ? 1'b0 : p_write + block_filled[p_write];
	p_request <= reset ? 1'b0 : p_request + read_request_ready;
     end

   // fifo_clock
   reg [8:0] 	  p_read_fclk = 0;
   wire [8:0] 	  p_write_fclk;
   wire 	  reset_fclk;
   gray_sync_3 sync0(.clock_in(clock), .in(p_write), .clock_out(fifo_clock), .out(p_write_fclk[8:6]));
   gray_sync_3 sync1(.clock_in(fifo_clock), .in(p_read_fclk[8:6]), .clock_out(clock), .out(p_read));
   sync sync_reset(.clock(clock), .in(reset), .out(reset_fclk));
   assign p_write_fclk[5:0] = 6'd0;
   wire 	  read_valid = fifo_read && (p_read_fclk != p_write_fclk);
   
   always @ (posedge fifo_clock)
     begin
	if(reset_fclk)
	  begin
	     fifo_read_valid <= 1'b0;
	     p_read_fclk <= 1'b0;
	  end
	else
	  begin
	     fifo_read_data <= fifo_bram[p_read_fclk];
	     p_read_fclk <= p_read_fclk + read_valid;
	     fifo_read_valid <= read_valid;
	  end
     end
endmodule
