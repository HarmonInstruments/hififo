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
   wire interrupt = 0;
   
   // AXI
   reg 	t_tready;
   wire [63:0] t_tdata;
   wire        t_1dw;
   wire        t_tlast;
   wire        t_tvalid;
   reg 	       r_tvalid;
   reg 	       r_tlast;
   reg [63:0]  r_tdata;

   wire [63:0] tpc0_data;
   wire	       tpc0_write;
   wire        tpc0_ready;

   wire [63:0] fpc0_data;
   wire	       fpc0_read;
   wire        fpc0_valid;
     
   initial begin
      $dumpfile("dump.vcd");
      $dumpvars(0, dut, sequencer);
      $from_myhdl(clock, reset, t_tready, r_tvalid, r_tlast, r_tdata);
      $to_myhdl(interrupt, t_tdata, t_1dw, t_tlast, t_tvalid);
   end
   
   hififo_pcie #(.ENABLE(8'b00010001)) dut
     (.pci_exp_txp(),
      .pci_exp_txn(),
      .pci_exp_rxp(4'b0),
      .pci_exp_rxn(4'b0),
      .sys_clk_p(1'b0),
      .sys_clk_n(1'b0),
      .sys_rst_n(1'b1),
      .clock(),
      //.pci_reset(),
      // FIFOs
      .fifo_clock({8{clock}}),
      .fifo_reset(),
      .tpc0_data(tpc0_data),
      .tpc0_write(tpc0_write),
      .tpc0_ready(tpc0_ready),
      .fpc0_data(fpc0_data),
      .fpc0_read(fpc0_read),
      .fpc0_valid(fpc0_valid)
      );

   assign dut.pcie_core_wrap.s_axis_tx_tready = t_tready;
   assign t_tdata = dut.pcie_core_wrap.s_axis_tx_tdata;
   assign t_tlast = dut.pcie_core_wrap.s_axis_tx_tlast;
   assign t_tvalid = dut.pcie_core_wrap.s_axis_tx_tvalid;

   assign dut.pcie_core_wrap.m_axis_rx_tvalid = r_tvalid;
   assign dut.pcie_core_wrap.m_axis_rx_tlast = r_tlast;
   assign dut.pcie_core_wrap.m_axis_rx_tdata = r_tdata;
   
   assign dut.pcie_core_wrap.clock = clock;
   assign dut.pcie_core_wrap.pci_reset = reset;

   sequencer sequencer
     (.clock(clock),
      .reset(reset),
      .fpc_read(fpc0_read),
      .fpc_valid(fpc0_valid),
      .fpc_data(fpc0_data),
      .tpc_ready(tpc0_ready),
      .tpc_write(tpc0_write),
      .tpc_data(tpc0_data));
      
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
        //
   output [15:0] pci_id,
   input 	 interrupt,
   output 	 interrupt_rdy,
   input [3:0] 	 interrupt_num,
   output [2:0]  interrupts_enabled,
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
   
endmodule