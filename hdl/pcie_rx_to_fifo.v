`timescale 1ns / 1ps

module pcie_rx_to_fifo
  (
   input 	     clock,
   input 	     reset,
   // PIO control
   input 	     pio_write_valid,
   input [12:0]      pio_write_address,
   input [63:0]      pio_write_data,
   // read completion
   input 	     completion_valid,
   input [63:0]      completion_data,
   // read request
   output reg 	     read_request_valid = 0,
   output [63:0]     read_request_address,
   // status
   output reg 	     fifo_interrupt_match = 0,
   output reg 	     fifo_interrupt_done = 0,
   output reg [17:0] fifo_block_count, // number of 512 byte blocks transferred
   // FIFO
   input 	     fifo_clock, // for all FIFO signals
   input 	     fifo_read,
   output [63:0]     fifo_read_data,
   output 	     fifo_empty
   );
   
   wire 	     fifo_almost_full;
   wire [63:0] 	     fifo_dout;
   reg 		     fifo_disable = 0;
   reg [41:0] 	     fifo_page_table[0:31];
   reg [41:0] 	     fifo_page_table_oreg;
   reg [17:0] 	     fifo_block_count_end;
   reg [17:0] 	     fifo_block_count_match;
   assign 	     read_request_address = {fifo_page_table_oreg, fifo_block_count[12:0], 9'd0};
   reg [6:0] 	     state = 0;
   reg 		     block_done = 0;
   reg 		     count_end = 0;
   reg 		     fifo_active = 0;
   
   always @ (posedge clock)
     begin
	count_end <= (fifo_block_count == fifo_block_count_end);
	block_done <= completion_valid && (state == 60); // receiving the last TLP of the request
	if(pio_write_valid && (pio_write_address == 11))
	  fifo_block_count_end <= pio_write_data[26:9];	
	fifo_disable <= pio_write_valid && (pio_write_address == 12);
	if(pio_write_valid && (pio_write_address == 13))
	  fifo_block_count_match <= pio_write_data[26:9];
	
	if(pio_write_valid && pio_write_address[12:5] == 2) // 64
	  fifo_page_table[pio_write_address[4:0]] <= pio_write_data[63:22];
	fifo_page_table_oreg <= fifo_page_table[fifo_block_count[17:13]];
	
	read_request_valid <= (state == 2);
	fifo_active <= state != 0;
		
	if(reset || fifo_disable)
	  state <= 7'd0;
	else if(state == 0)
	  state <= (pio_write_valid && (pio_write_address == 11));
	else if(state == 1)
	  state <= fifo_almost_full ? 7'd1 : 7'd2;
	else if(state == 2)
	  state <= 7'd3;
	else if(state < 67)
	  state <= state + completion_valid;
	else if(state == 67)
	  state <= count_end ? 7'd0 : 7'd1;
	else
	  state <= 7'd0;
	
	fifo_block_count <= (state == 0) ? 18'd0 : fifo_block_count + block_done;
	
	fifo_interrupt_done <= count_end && (state == 67);
	fifo_interrupt_match <= (fifo_block_count == fifo_block_count_match) && block_done;
     end

   FIFO_DUALCLOCK_MACRO  
     #(.ALMOST_EMPTY_OFFSET(9'h020),  // Sets the almost empty threshold
       .ALMOST_FULL_OFFSET(9'h050),  // Sets almost full threshold
       .DATA_WIDTH(64),
       .DEVICE("7SERIES"),
       .FIFO_SIZE ("36Kb"), // "18Kb" or "36Kb"
       .FIRST_WORD_FALL_THROUGH ("TRUE")
       )
   fifo_dma_read
     (.ALMOSTEMPTY(),
      .ALMOSTFULL(fifo_almost_full),
      .DO(fifo_read_data),
      .EMPTY(fifo_empty), .FULL(),
      .RDCOUNT(), .RDERR(),
      .WRCOUNT(), .WRERR(),
      .DI(completion_data),
      .RDCLK(fifo_clock),
      .RDEN(fifo_read),
      .RST(reset),
      .WRCLK(clock),
      .WREN(completion_valid)
      );
   
endmodule
