`timescale 1ns / 1ps

module pcie_dma_write_fifo
  (
   input 	     clock,
   input 	     reset,
   // PIO control
   input 	     pio_write_valid,
   input [12:0]      pio_write_address,
   input [63:0]      pio_write_data,
   // status
   output reg 	     interrupt_match = 0,
   output reg 	     interrupt_fifo = 0,
   output reg 	     active = 0,
   output reg [23:0] block_count, // number of 128 byte blocks transmitted
   // FIFO
   input 	     fifo_clock, // for all FIFO signals
   input 	     fifo_write_valid,
   input [64:0]      fifo_write_data, // bit 64 is end, 65 is interrupt
   output reg 	     fifo_ready, // minimum 32 positions available
   // interface to pcie_tx module
   output reg 	     write_request_valid = 0,
   output reg [63:0] write_request_data = 0,
   output [63:0]     write_request_address,
   input 	     write_request_ready, // pulses 16 times to read request data
   input 	     write_request_ack // pulses high once to indicate the header is complete
   );

   wire 	     fifo_almost_full;
   wire 	     fifo_almost_empty;
   wire [64:0] 	     fifo_dout;
   reg 		     pio_disable = 1'b0;
   reg 		     pio_enable = 1'b0;
   
   reg [23:0] 	     block_count_match;
   reg [41:0] 	     page_table[0:511];
   reg [41:0] 	     page_table_oreg1;
   reg [41:0] 	     page_table_oreg2;
   assign write_request_address = {page_table_oreg2, block_count[14:0], 7'd0};
      
   always @ (posedge clock)
     begin
	pio_enable <= pio_write_valid && (pio_write_address == 8);
	pio_disable <= pio_write_valid && (pio_write_address == 9);
	active <= (pio_disable || reset) ? 1'b0 : (active || pio_enable);
	
	fifo_ready <= ~fifo_almost_full;
	interrupt_match <= (block_count == block_count_match) && write_request_ack;
	interrupt_fifo <= fifo_dout[64] && write_request_ready;
	
	block_count <= active ? block_count + write_request_ack : 1'b0;
	if(pio_write_valid && pio_write_address[12:9] == 1)
	  page_table[pio_write_address[8:0]] <= pio_write_data[63:22];
	page_table_oreg1 <= page_table[block_count[23:15]];
	page_table_oreg2 <= page_table_oreg1;
	if(pio_write_valid && (pio_write_address == 10))
	  block_count_match <= pio_write_data[30:5];
	if(active && ~fifo_almost_empty && ~write_request_valid)
	  write_request_valid <= 1'b1;
	else if(write_request_ack)
	  write_request_valid <= 1'b0;
	if(write_request_ready)
	  write_request_data <= fifo_dout[63:0];
     end
   
   FIFO_DUALCLOCK_MACRO  
     #(.ALMOST_EMPTY_OFFSET(9'h010),  // Sets the almost empty threshold
       .ALMOST_FULL_OFFSET(9'h020),  // Sets almost full threshold
       .DATA_WIDTH(65),
       .DEVICE("7SERIES"),
       .FIFO_SIZE ("36Kb"), // "18Kb" or "36Kb"
       .FIRST_WORD_FALL_THROUGH ("TRUE")
       )
   fifo_dma_write_inst
     (.ALMOSTEMPTY(fifo_almost_empty),
      .ALMOSTFULL(fifo_almost_full),
      .DO(fifo_dout),
      .EMPTY(), .FULL(),
      .RDCOUNT(), .RDERR(),
      .WRCOUNT(), .WRERR(),
      .DI(fifo_write_data),
      .RDCLK(clock),
      .RDEN(write_request_ready),
      .RST(~active),
      .WRCLK(fifo_clock),
      .WREN(fifo_write_valid)
      );
   
endmodule
