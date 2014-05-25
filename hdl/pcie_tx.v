`timescale 1ns / 1ps

module pcie_tx
  (
   input 	     clock,
   input 	     reset,
   input [15:0]      pcie_id,
   // PIO control
   input 	     pio_write_valid,
   input [12:0]      pio_write_address,
   input [63:0]      pio_write_data,
   // read completion
   input 	     read_completion_valid,
   input [23:0]      read_completion_rid_tag,
   input [3:0] 	     read_completion_lower_addr,
   input [63:0]      read_completion_data,
   output reg 	     read_completion_ready = 0,
   // status
   output reg 	     fifo_interrupt_match = 0,
   output reg 	     fifo_interrupt_flag = 0,
   output reg 	     fifo_active = 0,
   output reg [19:0] fifo_block_count, // number of 128 byte blocks transmitted
   // FIFO
   input 	     fifo_clock, // for all FIFO signals
   input 	     fifo_write_valid,
   input [64:0]      fifo_write_data, // bit 64 is end, 65 is interrupt
   output reg 	     fifo_ready, // minimum 32 positions available
   // read request
   input 	     read_request_valid,
   input [63:0]      read_request_address,
   input [7:0] 	     read_request_tag,
   output reg 	     read_request_ready = 0,
   // AXI stream to PCI Express core
   input 	     axis_tx_tready,
   output reg [63:0] axis_tx_tdata = 0,
   output reg 	     axis_tx_1dw = 0,
   output reg 	     axis_tx_tlast = 0,
   output reg 	     axis_tx_tvalid = 0
   );

   reg [5:0] 	     tx_state = 0;

   wire 	     fifo_almost_full;
   wire 	     fifo_almost_empty;
   wire [64:0] 	     fifo_dout;
   reg 		     fifo_disable = 1'b0;
   reg 		     fifo_enable = 1'b0;
   reg [19:0] 	     fifo_block_count_match;
   reg [41:0] 	     fifo_page_table[0:31];
   reg [41:0] 	     fifo_page_table_oreg;
   wire 	     fifo_read = axis_tx_tready && (tx_state > 5) && (tx_state < 22); // pulses high 16 times, once for each word, 1 clock early
   wire 	     fifo_block_done = (tx_state == 23) && axis_tx_tready;
   wire [63:0] 	     write_request_address = {fifo_page_table_oreg, fifo_block_count[14:0], 7'd0};
   wire 	     write_request_is_32_bit = write_request_address[63:32] == 0;
   reg 		     write_request_valid;
   reg [63:0] 	     write_request_data;
   wire [63:0] 	     write_request_data_swapped;
   reg [63:0] 	     write_request_data_q;

   endian_swap endian_swap_wr4(.din(write_request_data[31: 0]), .dout(write_request_data_swapped[31: 0]));
   endian_swap endian_swap_wr5(.din(write_request_data[63:32]), .dout(write_request_data_swapped[63:32]));
   always @ (posedge clock)
     begin
	fifo_enable <= pio_write_valid && (pio_write_address == 8);
	fifo_disable <= pio_write_valid && (pio_write_address == 9);
	if(pio_write_valid && (pio_write_address == 10))
	  fifo_block_count_match <= pio_write_data[30:5];
	fifo_active <= (fifo_disable || reset) ? 1'b0 : (fifo_active || fifo_enable);
	fifo_ready <= ~fifo_almost_full;
	fifo_block_count <= fifo_active ? fifo_block_count + fifo_block_done : 1'b0;
	if(pio_write_valid && pio_write_address[12:9] == 1)
	  fifo_page_table[pio_write_address[4:0]] <= pio_write_data[63:22];
	fifo_page_table_oreg <= fifo_page_table[fifo_block_count[19:15]];
	fifo_interrupt_match <= (fifo_block_count == fifo_block_count_match) && fifo_block_done;
	fifo_interrupt_flag <= fifo_dout[64] && fifo_read;
	if(axis_tx_tready)
	  write_request_data_q <= write_request_data_swapped;
	if(fifo_read)
	  write_request_data <= fifo_dout[63:0];
     end
   
   wire [31:0] 	     read_completion_dw3;
   wire [31:0] 	     read_completion_dw4;
   endian_swap endian_swap_rc3(.din(read_completion_data[31: 0]), .dout(read_completion_dw3));
   endian_swap endian_swap_rc4(.din(read_completion_data[63:32]), .dout(read_completion_dw4));

   reg 		     read_request_is_32_bit = 0;
   wire [31:0] 	     read_request_dw0 = read_request_is_32_bit ? {1'b0, 7'b0000000, 24'd128} : {1'b0, 7'b0100000, 24'd128}; // 512 byte read request
   wire [31:0] 	     read_request_dw2 = read_request_is_32_bit ? read_request_address[31:0] : read_request_address[63:32];
       
   always @(posedge clock)
     begin
	read_request_is_32_bit <= read_request_address[63:32] == 0;
	read_completion_ready <= axis_tx_tready && (tx_state == 3);
	read_request_ready <= axis_tx_tready && (tx_state == 5);
	axis_tx_tvalid <= tx_state != 0;
	axis_tx_1dw <= (tx_state == 3) || ((tx_state == 5) && read_request_is_32_bit) || ((tx_state == 23) && write_request_is_32_bit);
	axis_tx_tlast <= (tx_state == 3) || (tx_state == 5) || (tx_state == 23);
	case(tx_state)
	  0: axis_tx_tdata <= 64'h0;
	  1: axis_tx_tdata <= {{pcie_id, 16'd8}, 32'h4A000002}; // completion, 2 DW
	  2: axis_tx_tdata <= {read_completion_dw3, {read_completion_rid_tag, 1'b0, read_completion_lower_addr, 3'd0 }};
	  3: axis_tx_tdata <= {32'h0,               read_completion_dw4};
	  4: axis_tx_tdata <= {{pcie_id, read_request_tag, 8'hFF}, read_request_dw0};
	  5: axis_tx_tdata <= {read_request_address[31:0], read_request_dw2};
	  6: axis_tx_tdata <= {{pcie_id, 16'h00FF}, {1'b0, 1'b1, ~write_request_is_32_bit, 5'b00000, 24'd32}}; // {write_req_dw1, write_req_dw0}
	  7: axis_tx_tdata <= write_request_is_32_bit ? {write_request_data_swapped[31:0], write_request_address[31:0]} : {write_request_address[31:0], write_request_address[63:32]};
	  default: axis_tx_tdata <= write_request_is_32_bit ? {write_request_data_swapped[31:0], write_request_data_q[63:32]} : write_request_data_q;
	endcase
	if(reset)
	  tx_state <= 5'd0;
	else if(tx_state == 0)
	  begin
	     if(read_completion_valid & ~read_completion_ready)
	       tx_state <= 5'd1;
	     else if(read_request_valid & ~read_request_ready)
	       tx_state <= 5'd4;
	     else if(~fifo_almost_empty & fifo_active)
	       tx_state <= 5'd6;
	     else
	       tx_state <= 5'd0;
	  end
	else if(axis_tx_tready)
	  begin
	     if(tx_state == 3 || tx_state == 5 || tx_state == 23)
	       tx_state <= 5'd0;
	     else
	       tx_state <= tx_state + 1'b1;
	  end
     end

   FIFO_DUALCLOCK_MACRO  
     #(.ALMOST_EMPTY_OFFSET(9'h010),  // Sets the almost empty threshold
       .ALMOST_FULL_OFFSET(9'h020),  // Sets almost full threshold
       .DATA_WIDTH(65),
       .DEVICE("7SERIES"),
       .FIFO_SIZE ("36Kb"), // "18Kb" or "36Kb"
       .FIRST_WORD_FALL_THROUGH ("TRUE")
       )
   fifo_dma_write
     (.ALMOSTEMPTY(fifo_almost_empty),
      .ALMOSTFULL(fifo_almost_full),
      .DO(fifo_dout),
      .EMPTY(), .FULL(),
      .RDCOUNT(), .RDERR(),
      .WRCOUNT(), .WRERR(),
      .DI(fifo_write_data),
      .RDCLK(clock),
      .RDEN(fifo_read),
      .RST(~fifo_active),
      .WRCLK(fifo_clock),
      .WREN(fifo_write_valid)
      );
   
endmodule

module endian_swap
  (
   input [31:0] din,
   output [31:0] dout
   );
   assign dout[7:0] = din[31:24];
   assign dout[15:8] = din[23:16];
   assign dout[23:16] = din[15:8];
   assign dout[31:24] = din[7:0];
endmodule