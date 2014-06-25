module hififo_pcie
  (
   // from core
   input 	 clock,
   input 	 pci_reset,
   input [15:0]  pci_id,
   // to core
   output reg 	 interrupt_out = 0,
   // AXI from core
   input 	 s_axis_tx_tready,
   output [63:0] s_axis_tx_tdata,
   output 	 s_axis_tx_1dw,
   output 	 s_axis_tx_tlast,
   output 	 s_axis_tx_tvalid,
   // AXI from core
   input 	 m_axis_rx_tvalid,
   input 	 m_axis_rx_tlast,
   input [63:0]  m_axis_rx_tdata, 
   // FIFOs
   input 	 tpc0_clock,
   output 	 tpc0_reset,
   input [63:0]  tpc0_data,
   input 	 tpc0_write,
   output 	 tpc0_ready,
   input 	 fpc0_clock,
   output 	 fpc0_reset,
   output [63:0] fpc0_data,
   input 	 fpc0_read,
   output 	 fpc0_empty,
   // PIO
   output 	 pio_write_valid,
   output [63:0] pio_write_data,
   output [12:0] pio_address
   );
  
   assign {tpc0_reset,fpc0_reset} = 2'b00;
   
   // from RX module
   wire 	rx_rc_valid;
   wire [5:0] 	rx_rc_index;
   wire [7:0] 	rx_rc_tag;
   wire 	rx_rr_valid;
   wire [23:0] 	rx_rr_rid_tag;
   wire 	rx_wr_valid;
   wire [63:0] 	rx_data;
   wire [12:0] 	rx_address;

   assign pio_write_data = rx_data;
   assign pio_write_valid = rx_wr_valid;
   assign pio_address = rx_address;
      
   // read completion request to TX module
   wire 	tx_rc_ready;
   reg 		tx_rc_valid = 0;
   reg [23:0] 	tx_rc_rid_tag = 0;
   reg [3:0] 	tx_rc_lower_addr = 0;
   reg [63:0] 	tx_rc_data = 0;
   
   wire 	fpc_rr_valid;
   wire 	fpc_rr_ready;
   wire [7:0] 	fpc_rr_tag = 0;
   wire [63:0] 	fpc_rr_address = 0;
           
   reg 		read_done = 0;
   reg [63:0] 	read_data = 0;
   reg [63:0] 	ldata = 0;
   reg [15:0] 	cpld_count = 0;
   reg [15:0] 	write_count = 0;
   reg [15:0] 	read_count = 0;
      
   reg [3:0] 	interrupt_latch = 0;
   reg [3:0] 	interrupt_enable = 0;
   wire [3:0] 	interrupt = 0;
   
   // PIO control
   always @ (posedge clock)
     begin
	if(rx_wr_valid)
	  begin
	     case(rx_address)
	       0: interrupt_enable <= rx_data[3:0];
	     endcase
	  end
	if(rx_rc_valid)
	  ldata <= rx_data;

	cpld_count <= cpld_count + rx_rc_valid;
	read_count <= read_count + rx_rr_valid;
	write_count <= write_count + rx_wr_valid;
	interrupt_latch <= (rx_rr_valid && rx_address == 0) ? 1'b0 : interrupt_latch | interrupt;
	interrupt_out <= (interrupt & interrupt_enable) != 0;
	
	if(rx_rr_valid)
	  begin
	     case(rx_address)
	       14'h0000: read_data <= {8'd1, 28'd0, interrupt_latch};
	       14'h0002: read_data <= {cpld_count, write_count, read_count};
	       //14'h0003: read_data <= {write_fifo_block_count, 7'd0};
	       //14'h0004: read_data <= {read_fifo_block_count, 9'd0};
	       14'h0007: read_data <= ldata;
	       default: read_data <= 64'h0;
	     endcase
	     tx_rc_rid_tag <= rx_rr_rid_tag;
	     tx_rc_lower_addr <= rx_address[3:0];
	  end
	read_done <= rx_rr_valid;
	if(read_done)
	  begin
	     tx_rc_data <= read_data;
	     tx_rc_valid <= 1'b1;
	  end
	else if(tx_rc_ready)
	  tx_rc_valid <= 1'b0;
     end
   
   pcie_from_pc_fifo fpc0_fifo
     (.clock(clock),
      .reset(pci_reset),
      .channel(3'd0),
      // read completion
      .completion_valid(rx_rc_valid),
      .completion_tag(rx_rc_tag),
      .completion_index(rx_rc_index),
      .completion_data(rx_data),
      // read request
      .read_request_valid(fpc_rr_valid),
      .read_request_ready(fpc_rr_ready),
      // FIFO
      .fifo_clock(clock),
      .fifo_read(fpc0_read),
      .fifo_read_data(fpc0_data),
      .fifo_read_valid(fpc0_empty)
      );

   wire        tx_wr_valid = 0;
   wire [63:0] tx_wr_addr = 0;
   wire        tx_wr_ready;
   wire [63:0] tx_wr_data;
      
   hififo_tpc_fifo tpc0_fifo
     (.clock(clock),
      .reset(tpc0_reset),
      .channel(3'd0),
      .ready_for_read(),
      .read_address(),
      .read_data(tx_wr_data),
      .read(),
      .fifo_clock(tpc0_clock),
      .fifo_data(tpc0_data),
      .fifo_write(tpc0_write),
      .fifo_ready(tpc0_ready)
      );
   
   pcie_rx rx
     (.clock(clock),
      .reset(pci_reset),
      // outputs
      .write_valid(rx_wr_valid),
      .read_valid(rx_rr_valid),
      .completion_valid(rx_rc_valid),
      .completion_index(rx_rc_index),
      .completion_tag(rx_rc_tag),
      .data(rx_data),
      .address(rx_address),
      .rid_tag(rx_rr_rid_tag),
      // AXI stream from PCIE core
      .tvalid(m_axis_rx_tvalid),
      .tlast(m_axis_rx_tlast),
      .tdata(m_axis_rx_tdata)
      );		     
  
   pcie_tx tx
     (.clock(clock),
      .reset(pci_reset),
      .pcie_id(pci_id),
      // read completion (rc)
      .rc_valid(tx_rc_valid),
      .rc_rid_tag(tx_rc_rid_tag), // 24
      .rc_lower_addr(tx_rc_lower_addr),// 4
      .rc_data(tx_rc_data),
      .rc_ready(tx_rc_ready),
      // read request (rr)
      .rr_valid(fpc_rr_valid),
      .rr_ready(fpc_rr_ready),
      .rr_addr(fpc_rr_address),
      .rr_tag(fpc_rr_tag),
      // write request (wr)
      .wr_valid(tx_wr_valid),
      .wr_addr(tx_wr_addr),
      .wr_ready(tx_wr_ready),
      .wr_data(tx_wr_data),
      // AXI stream to PCI Express core
      .tx_tready(s_axis_tx_tready),
      .tx_tdata(s_axis_tx_tdata),
      .tx_1dw(s_axis_tx_1dw),
      .tx_tlast(s_axis_tx_tlast),
      .tx_tvalid(s_axis_tx_tvalid)
   );
endmodule

