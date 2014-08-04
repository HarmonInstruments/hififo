/* 
 * HIFIFO: Harmon Instruments PCI Express to FIFO
 * Copyright (C) 2014 Harmon Instruments, LLC
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/
 */

module hififo_pcie
  (
   // from core
   input 	 clock,
   input 	 pci_reset,
   input [15:0]  pci_id,
   // to core
   output reg 	 interrupt_out,
   // AXI to core
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
   output 	 fpc0_valid
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

   // read completion request to TX module
   wire [31:0] 	tx_rc_dw2;
   reg [31:0] 	tx_rc_data;
   reg 		tx_rc_done = 0;
   
   wire 	tx_rr_valid;
   wire 	tx_rr_ready;
   wire [7:0] 	tx_rr_tag;
   wire [63:0] 	tx_rr_addr;

   wire [3:0] 	rr_mux_valid;
   wire [3:0] 	rr_mux_ready;
   wire [63:0] 	rr_mux_addr0;
        
   wire 	tx_wr_valid;
   wire 	tx_wr_ready;
   wire [63:0] 	tx_wr_addr;
   wire [63:0] 	tx_wr_data;
  
   reg [1:0] 	reset_fifo_0 = 3;
   reg [1:0] 	reset_fifo = 3;
   
   wire [31:0] 	fpc_status, tpc_status;
         
   always @ (posedge clock)
     begin
	reset_fifo <= pci_reset ? 2'b11 : reset_fifo_0;
	interrupt_prev <= interrupt;
	interrupt_out <= (interrupt_mask & (interrupt ^ interrupt_prev)) != 0;
	interrupt_status <= pci_reset | (rx_rr_valid && (rx_address == 0)) ? 1'b0 : interrupt_status | interrupt;
	case({~rx_wr_valid, rx_address})
	  0: interrupt_mask <= rx_data[3:0];
	  8: reset_fifo_0 <= rx_data[1:0];
	endcase	
	case({~rx_rr_valid, rx_address})
	  0: tx_rc_data <= interrupt_status;
	  1: tx_rc_data <= 32'h0101;
	  2: tx_rc_data <= tpc_status;
	  5: tx_rc_data <= fpc_status;
	endcase
	tx_rc_done <= rx_rr_valid;
     end

   assign interrupt[3] = 0;
   assign interrupt[1] = 0;
      
   pcie_from_pc_fifo fpc0_fifo
     (.clock(clock),
      .reset(reset_fifo[0]),
      .interrupt(interrupt[2]),
      .status(fpc_status),
      .fifo_number(2'd0),
      // PIO
      .pio_wvalid(rx_wr_valid && (rx_address[12:4] == 0)),
      .pio_wdata(rx_data),
      .pio_addr(rx_address[3:0]),
      // read completion
      .rc_valid(rx_rc_valid),
      .rc_tag(rx_rc_tag),
      .rc_index(rx_rc_index),
      .rc_data(rx_data),
      // read request
      .rr_valid(rr_mux_valid[0]),
      .rr_ready(rr_mux_ready[0]),
      .rr_addr(rr_mux_addr0),    
      // FIFO
      .fifo_clock(fifo_clock),
      .fifo_read(fpc0_read),
      .fifo_read_data(fpc0_data),
      .fifo_read_valid(fpc0_valid)
      );

   assign 	rr_mux_valid[3:1] = 0;
         
   hififo_tpc_fifo tpc0_fifo
     (.clock(clock),
      .reset(reset_fifo[1]),
      .interrupt(interrupt[0]),
      .status(tpc_status),
      // PIO
      .pio_wvalid(rx_wr_valid),
      .pio_wdata(rx_data),
      .pio_addr(rx_address),
      // write request to TX
      .wr_valid(tx_wr_valid),
      .wr_ready(tx_wr_ready),
      .wr_addr(tx_wr_addr),
      .wr_data(tx_wr_data),
      // user FIFO
      .fifo_clock(fifo_clock),
      .fifo_data(tpc0_data),
      .fifo_write(tpc0_write),
      .fifo_ready(tpc0_ready)
      );
  
   fpc_rr_mux fpc_rr_mux
     (
      .clock(clock),
      .reset(pci_reset),
      // PIO
      .pio_wvalid(rx_wr_valid),
      .pio_wdata(rx_data),
      .pio_addr(rx_address),
      // read request in
      .rr_valid(rr_mux_valid),
      .rr_ready(rr_mux_ready),
      .rr0_addr(rr_mux_addr0),
      .rr1_addr(1'b0),
      .rr2_addr(1'b0),
      .rr3_addr(1'b0),
      // rr request multiplexed
      .rrm_valid(tx_rr_valid),
      .rrm_addr(tx_rr_addr),
      .rrm_tag(tx_rr_tag),
      .rrm_ready(tx_rr_ready)
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
      .wr_data(tx_wr_data),
      // AXI stream to PCI Express core
      .tx_tready(s_axis_tx_tready),
      .tx_tdata(s_axis_tx_tdata),
      .tx_1dw(s_axis_tx_1dw),
      .tx_tlast(s_axis_tx_tlast),
      .tx_tvalid(s_axis_tx_tvalid)
   );
endmodule

