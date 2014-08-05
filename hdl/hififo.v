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

`timescale 1ns/1ps

module hififo_pcie
  (
   // IO pins
   output [3:0]  pci_exp_txp,
   output [3:0]  pci_exp_txn,
   input [3:0] 	 pci_exp_rxp,
   input [3:0] 	 pci_exp_rxn,
   input 	 sys_clk_p,
   input 	 sys_clk_n,
   input 	 sys_rst_n,
   // from core
   output 	 clock,
   // FIFOs
   input [7:0] 	 fifo_clock,
   output [7:0]  fifo_reset,
   input [63:0]  tpc0_data,
   input 	 tpc0_write,
   output 	 tpc0_ready,
   output [63:0] fpc0_data,
   input 	 fpc0_read,
   output 	 fpc0_valid
   );
   
   parameter ENABLE = 8'b00010001;
   
   assign fifo_reset = 8'd0;
   
   // interrupts
   wire 	 interrupt;
   wire 	 interrupt_rdy;
   wire [3:0] 	 interrupt_num;
   wire [2:0] 	 interrupts_enabled;
   
   wire [15:0] 	 pci_id;
   wire 	 pci_reset;
      
   // from RX module
   wire 	rx_rc_valid;
   wire [5:0] 	rx_rc_index;
   wire [7:0] 	rx_rc_tag;
   wire 	rx_rr_valid;
   wire 	rx_wr_valid;
   wire [63:0] 	rx_data;
   wire [10:0] 	rx_address;

   // read completion request to TX module
   wire [31:0] 	tx_rc_dw2;
   reg [31:0] 	tx_rc_data;
   reg 		tx_rc_done = 0;
   
   wire 	tx_rr_valid;
   wire 	tx_rr_ready;
   wire [7:0] 	tx_rr_tag;
   wire [54:0] 	tx_rr_addr;

   wire [3:0] 	rr_mux_valid;
   wire [3:0] 	rr_mux_ready;
   wire [2:0] 	rr_tag_low0;
        
   wire 	tx_wr_valid;
   wire 	tx_wr_ready;
   wire [65:0] 	tx_wr_data0;

   wire [7:0] 	r_valid;
   wire [60:0] 	r_addr;
   wire [18:0] 	r_count;
   wire [7:0] 	r_ready;
   
   reg [1:0] 	reset_fifo_0 = 3;
   reg [1:0] 	reset_fifo = 3;
   
   wire [31:0] 	status[0:7];
         
   always @ (posedge clock)
     begin
	reset_fifo <= pci_reset ? 2'b11 : reset_fifo_0;
	case({~rx_wr_valid, rx_address})
	  8: reset_fifo_0 <= rx_data[1:0];
	endcase
	if(rx_rr_valid)
	  case(rx_address)
	    0:  tx_rc_data <= 1'b0;
	    1:  tx_rc_data <= 32'h0101;
	    32: tx_rc_data <= status[0];
	    48: tx_rc_data <= status[4];
	  endcase
	tx_rc_done <= rx_rr_valid;
     end
      
   pcie_from_pc_fifo fpc0_fifo
     (.clock(clock),
      .reset(reset_fifo[0]),
      .status(status[0]),
      .fifo_number(2'd0),
      // read completion
      .rc_valid(rx_rc_valid),
      .rc_tag(rx_rc_tag),
      .rc_index(rx_rc_index),
      .rc_data(rx_data),
      // read request
      .rr_valid(rr_mux_valid[0]),
      .rr_ready(rr_mux_ready[0]),
      .rr_tag_low(rr_tag_low0),    
      // FIFO
      .fifo_clock(fifo_clock[0]),
      .fifo_read(fpc0_read),
      .fifo_read_data(fpc0_data),
      .fifo_read_valid(fpc0_valid)
      );

   assign 	rr_mux_valid[3:1] = 0;
         
   hififo_tpc_fifo tpc0_fifo
     (.clock(clock),
      .reset(reset_fifo[1]),
      .pci_id(pci_id),
      .status(status[4]),
      // request unit
      .r_valid(r_valid[4]),
      .r_addr(r_addr),
      .r_count(r_count),
      .r_ready(r_ready[4]),
      // write request to TX
      .wr_valid(tx_wr_valid),
      .wr_ready(tx_wr_ready),
      .wr_data(tx_wr_data0),
      // user FIFO
      .fifo_clock(fifo_clock[4]),
      .fifo_data(tpc0_data),
      .fifo_write(tpc0_write),
      .fifo_ready(tpc0_ready)
      );

   assign r_ready[7:5] = 3'd0;
   
   hififo_request hififo_request
     (
      .clock(clock),
      .reset(pci_reset),
      // PIO
      .pio_wvalid(rx_wr_valid),
      .pio_wdata(rx_data),
      .pio_addr(rx_address),
      // request unit
      .r_valid(r_valid),
      .r_addr(r_addr),
      .r_count(r_count),
      .r_ready(r_ready)
     );
   
   fpc_rr_mux fpc_rr_mux
     (
      .clock(clock),
      .reset(pci_reset),
      // request unit
      .r_valid(r_valid[3:0]),
      .r_addr(r_addr),
      .r_count(r_count),
      .r_ready(r_ready[3:0]),
      // read request in
      .rr_valid(rr_mux_valid),
      .rr_ready(rr_mux_ready),
      .rr0_tag_low(rr_tag_low0),
      .rr1_tag_low(1'b0),
      .rr2_tag_low(1'b0),
      .rr3_tag_low(1'b0),
      // rr request multiplexed
      .rrm_valid(tx_rr_valid),
      .rrm_addr(tx_rr_addr),
      .rrm_tag(tx_rr_tag),
      .rrm_ready(tx_rr_ready)
     );

   // AXI to core
   wire 	 s_axis_tx_tready;
   wire [63:0] 	 s_axis_tx_tdata;
   wire 	 s_axis_tx_1dw;
   wire 	 s_axis_tx_tlast;
   wire 	 s_axis_tx_tvalid;
   // AXI from core
   wire 	 m_axis_rx_tvalid;
   wire 	 m_axis_rx_tlast;
   wire [63:0] 	 m_axis_rx_tdata;
   
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
      .rr_addr({tx_rr_addr,9'd0}),
      .rr_ready(tx_rr_ready),
      .rr_tag(tx_rr_tag),
      // write request (wr)
      .wr_valid(tx_wr_valid),
      .wr_ready(tx_wr_ready),
      .wr_data0(tx_wr_data0),
      // AXI stream to PCI Express core
      .tx_tready(s_axis_tx_tready),
      .tx_tdata(s_axis_tx_tdata),
      .tx_1dw(s_axis_tx_1dw),
      .tx_tlast(s_axis_tx_tlast),
      .tx_tvalid(s_axis_tx_tvalid)
   );

   pcie_core_wrap pcie_core_wrap
     (.pci_exp_txp(pci_exp_txp),
      .pci_exp_txn(pci_exp_txn),
      .pci_exp_rxp(pci_exp_rxp),
      .pci_exp_rxn(pci_exp_rxn),
      .sys_clk_p(sys_clk_p),
      .sys_clk_n(sys_clk_n),
      .sys_rst_n(sys_rst_n),
      .clock(clock),
      .pci_id(pci_id),
      .interrupt(interrupt),
      .interrupt_num(interrupt_num),
      .interrupt_rdy(interrupt_rdy),
      .interrupts_enabled(interrupts_enabled),
      .pci_reset(pci_reset),
      .s_axis_tx_tready(s_axis_tx_tready),
      .s_axis_tx_tdata(s_axis_tx_tdata),
      .s_axis_tx_1dw(s_axis_tx_1dw),
      .s_axis_tx_tlast(s_axis_tx_tlast),
      .s_axis_tx_tvalid(s_axis_tx_tvalid),
      .m_axis_rx_tvalid(m_axis_rx_tvalid),
      .m_axis_rx_tlast(m_axis_rx_tlast),
      .m_axis_rx_tdata(m_axis_rx_tdata)
      );
   
endmodule

