module hififo_pcie
  (
   // from core
   input 	 clock,
   input 	 pci_reset,
   input [15:0]  pci_id,
   // to core
   output reg 	 interrupt_out,
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
   input 	 fifo_clock,
   output 	 tpc0_reset,
   input [63:0]  tpc0_data,
   input 	 tpc0_write,
   output 	 tpc0_ready,
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
   wire [3:0] 	 interrupt;
   reg [3:0] 	 interrupt_prev = 0;
   reg [3:0] 	 interrupt_mask = 0;
   reg [3:0] 	 interrupt_status = 0;
   
   // from RX module
   wire 	rx_rc_valid;
   wire [5:0] 	rx_rc_index;
   wire [7:0] 	rx_rc_tag;
   wire 	rx_rr_valid;
   wire 	rx_wr_valid;
   wire [63:0] 	rx_data;
   wire [12:0] 	rx_address;

   assign pio_write_data = rx_data;
   assign pio_write_valid = rx_wr_valid;
   assign pio_address = rx_address;
   
   // read completion request to TX module
   wire [31:0] 	tx_rc_dw2;
   reg [63:0] 	tx_rc_data;
   reg 		tx_rc_done = 0;
   
   wire 	tx_rr_valid;
   wire 	tx_rr_ready;
   wire [7:0] 	tx_rr_tag;
   wire [63:0] 	tx_rr_addr;
        
   wire 	tx_wr_valid;
   wire 	tx_wr_ready;
   wire [63:0] 	tx_wr_addr;
   
   wire 	tx_wrr_write;
   wire [8:0] 	tx_wrr_addr;
   wire [63:0] 	tx_wrr_data;

   reg [63:0] 	count = 0;
   reg [1:0] 	reset_fifo_0 = 3;
   reg [1:0] 	reset_fifo = 3;
   
   wire [63:0] 	fpc_status, tpc_status;
         
   always @ (posedge clock)
     begin
	reset_fifo <= pci_reset ? 3'b11 : reset_fifo_0;
	interrupt_prev <= interrupt;
	interrupt_out <= (interrupt_mask & (interrupt ^ interrupt_prev)) != 0;
	interrupt_status <= pci_reset | (rx_rr_valid && (rx_address == 0)) ? 1'b0 : interrupt_status | interrupt;
	count <= count + 1'b1;
	case({~rx_wr_valid, rx_address})
	  0: interrupt_mask <= rx_data[3:0];
	  8: reset_fifo_0 <= rx_data[1:0];
	endcase	
	case({~rx_rr_valid, rx_address})
	  0: tx_rc_data <= interrupt_status;
	  1: tx_rc_data <= count;
	  2: tx_rc_data <= tpc_status;
	  5: tx_rc_data <= fpc_status;
	endcase
	tx_rc_done <= rx_rr_valid;
     end
   
   pcie_from_pc_fifo fpc0_fifo
     (.clock(clock),
      .reset(reset_fifo[0]),
      .interrupt(interrupt[3:2]),
      .status(fpc_status),
      // PIO
      .pio_wvalid(rx_wr_valid),
      .pio_wdata(rx_data),
      .pio_addr(rx_address),
      // read completion
      .rc_valid(rx_rc_valid),
      .rc_tag(rx_rc_tag),
      .rc_index(rx_rc_index),
      .rc_data(rx_data),
      // read request
      .rr_valid(tx_rr_valid),
      .rr_ready(tx_rr_ready),
      .rr_tag(tx_rr_tag),
      .rr_addr(tx_rr_addr),    
      // FIFO
      .fifo_clock(fifo_clock),
      .fifo_read(fpc0_read),
      .fifo_read_data(fpc0_data),
      .fifo_read_valid(fpc0_empty)
      );
      
   hififo_tpc_fifo tpc0_fifo
     (.clock(clock),
      .reset(reset_fifo[1]),
      .interrupt(interrupt[1:0]),
      .status(tpc_status),
      // PIO
      .pio_wvalid(rx_wr_valid),
      .pio_wdata(rx_data),
      .pio_addr(rx_address),
      // write request ram
      .wrr_write(tx_wrr_write),
      .wrr_addr(tx_wrr_addr),
      .wrr_data(tx_wrr_data),
      // write request to TX
      .wr_valid(tx_wr_valid),
      .wr_ready(tx_wr_ready),
      .wr_addr(tx_wr_addr),
      // user FIFO
      .fifo_clock(fifo_clock),
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
      .rr_rc_dw2(tx_rc_dw2),
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
      .rc_done(tx_rc_done),
      .rc_dw2(tx_rc_dw2),
      .rc_data(tx_rc_data),
      // read request (rr)
      .rr_valid(tx_rr_valid),
      .rr_addr(tx_rr_addr),
      .rr_ready(tx_rr_ready),
      .rr_tag(tx_rr_tag),
      // write request (wr)
      .wr_valid(tx_wr_valid),
      .wr_addr(tx_wr_addr),
      .wr_ready(tx_wr_ready),
      // write request ram (wrr)
      .wrr_clock(fifo_clock),
      .wrr_write(tx_wrr_write),
      .wrr_data(tx_wrr_data),
      .wrr_addr(tx_wrr_addr),
      // AXI stream to PCI Express core
      .tx_tready(s_axis_tx_tready),
      .tx_tdata(s_axis_tx_tdata),
      .tx_1dw(s_axis_tx_1dw),
      .tx_tlast(s_axis_tx_tlast),
      .tx_tvalid(s_axis_tx_tvalid)
   );
endmodule

