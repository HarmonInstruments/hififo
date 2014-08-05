/* 
 * PCI Express to FIFO example
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

module vna_dsp
  (
   output [3:0]     pcie_txp,
   output [3:0]     pcie_txn,
   input [3:0] 	    pcie_rxp,
   input [3:0] 	    pcie_rxn,
   input 	    pcie_refclk_p,
   input 	    pcie_refclk_n,
   input 	    pcie_rst_n,
   output reg [3:0] led = 4'h5
   );
   
   wire 	    clock;
   
   reg [63:0] 	    tpc_data = 0;
   reg 		    tpc_write = 0;
   wire 	    tpc_ready;
   
   wire [63:0] 	    fpc_data;
   reg 		    fpc_read = 1'b0;
   wire 	    fpc_valid;
   
   hififo_pcie #(.ENABLE(8'b00010001)) hififo
     (.pci_exp_txp(pcie_txp),
      .pci_exp_txn(pcie_txn),
      .pci_exp_rxp(pcie_rxp),
      .pci_exp_rxn(pcie_rxn),
      .sys_clk_p(pcie_refclk_p),
      .sys_clk_n(pcie_refclk_n),
      .sys_rst_n(pcie_rst_n),
      .clock(clock),
      .fifo_clock({8{clock}}),
      .fifo_reset(),
      .tpc0_data(tpc_data),
      .tpc0_write(tpc_write),
      .tpc0_ready(tpc_ready),
      .fpc0_data(fpc_data),
      .fpc0_read(fpc_read),
      .fpc0_valid(fpc_valid)
      );
   
   always @ (posedge clock)
     begin
	if(fpc_valid)
	  led[3:0] <= fpc_data[3:0];
	fpc_read <= tpc_ready;
	tpc_write <= fpc_valid && fpc_read;
	tpc_data <= fpc_data;
     end
endmodule
