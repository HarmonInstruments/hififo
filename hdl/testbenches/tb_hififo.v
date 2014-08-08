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

module tb_hififo_pcie;
   reg clock = 0;
   reg reset = 1;
   wire interrupt;
   
   // AXI
   reg 	t_tready;
   wire [63:0] t_tdata;
   wire        t_1dw;
   wire        t_tlast;
   wire        t_tvalid;
   reg 	       r_tvalid;
   reg 	       r_tlast;
   reg [63:0]  r_tdata;
     
   initial begin
      $dumpfile("dump.vcd");
      $dumpvars(0);
      $from_myhdl(clock, reset, t_tready, r_tvalid, r_tlast, r_tdata);
      $to_myhdl(interrupt, t_tdata, t_1dw, t_tlast, t_tvalid);
   end
   
   vna_dsp dut
     (.pcie_txp(),
      .pcie_txn(),
      .pcie_rxp(4'b0),
      .pcie_rxn(4'b0),
      .pcie_refclk_p(1'b0),
      .pcie_refclk_n(1'b0),
      .pcie_rst_n(1'b1),
      .led());
   
   assign dut.hififo.pcie_core_wrap.s_axis_tx_tready = t_tready;
   assign t_tdata = dut.hififo.pcie_core_wrap.s_axis_tx_tdata;
   assign t_tlast = dut.hififo.pcie_core_wrap.s_axis_tx_tlast;
   assign t_tvalid = dut.hififo.pcie_core_wrap.s_axis_tx_tvalid;
   assign t_1dw = dut.hififo.pcie_core_wrap.s_axis_tx_1dw;

   assign dut.hififo.pcie_core_wrap.m_axis_rx_tvalid = r_tvalid;
   assign dut.hififo.pcie_core_wrap.m_axis_rx_tlast = r_tlast;
   assign dut.hififo.pcie_core_wrap.m_axis_rx_tdata = r_tdata;
   
   assign dut.hififo.pcie_core_wrap.clock = clock;
   assign dut.hififo.pcie_core_wrap.pci_reset = reset;

   assign interrupt = dut.hififo.pcie_core_wrap.interrupt & dut.hififo.pcie_core_wrap.interrupt_rdy;
      
endmodule

module pcie_core_wrap
  (
   // IO pins
   output [3:0]  pci_exp_txp,
   output [3:0]  pci_exp_txn,
   input [3:0] 	 pci_exp_rxp,
   input [3:0] 	 pci_exp_rxn,
   input 	 sys_clk_p,
   input 	 sys_clk_n,
   input 	 sys_rst_n,
   output [15:0] pci_id,
   input 	 interrupt,
   output reg 	 interrupt_rdy,
   output 	 pci_reset,
   output 	 clock,
        // AXI to core
   output 	 s_axis_tx_tready,
   input [63:0]  s_axis_tx_tdata,
   input 	 s_axis_tx_1dw,
   input 	 s_axis_tx_tlast,
   input 	 s_axis_tx_tvalid,
        // AXI from core
   output 	 m_axis_rx_tvalid,
   output 	 m_axis_rx_tlast,
   output [63:0] m_axis_rx_tdata
   );
   
   assign pci_id = 16'hDEAD;
   always @ (posedge clock)
     interrupt_rdy <= interrupt;
   
endmodule