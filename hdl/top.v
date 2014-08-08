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
    
   wire [63:0] 	    fpc_data;
   reg 		    fpc_read = 1'b0;
   
   wire [7:0] 	    fifo_ready, fifo_reset;
   wire [63:0] 	    seq_fpc_data, seq_tpc_data;
   wire 	    seq_read, seq_write;

   reg [63:0] 	    count = 0;
   reg 		    count_write = 0;

         
   hififo_pcie #(.ENABLE(8'b01110111)) hififo
     (.pci_exp_txp(pcie_txp),
      .pci_exp_txn(pcie_txn),
      .pci_exp_rxp(pcie_rxp),
      .pci_exp_rxn(pcie_rxn),
      .sys_clk_p(pcie_refclk_p),
      .sys_clk_n(pcie_refclk_n),
      .sys_rst_n(pcie_rst_n),
      .clock(clock),
      .fifo_clock({8{clock}}),
      .fifo_reset(fifo_reset),
      .fifo_ready(fifo_ready),
      .fifo_rw(~fifo_reset & {1'b1, count_write, seq_write, tpc_write, fifo_ready[3:2], seq_read, fpc_read}),
      .fifo_data_0(fpc_data),
      .fifo_data_1(seq_fpc_data),
      .fifo_data_2(),
      .fifo_data_3(),
      .fifo_data_4(tpc_data),
      .fifo_data_5(seq_tpc_data),
      .fifo_data_6(count),
      .fifo_data_7(64'h0)      
      );

   sequencer sequencer
     (.clock(clock),
      .reset(1'b0),
      .fpc_read(seq_read),
      .fpc_valid(fifo_ready[1]),
      .fpc_data(seq_fpc_data),
      .tpc_ready(fifo_ready[5]),
      .tpc_write(seq_write),
      .tpc_data(seq_tpc_data));
   
   
   always @ (posedge clock)
     begin
	count <= count + fifo_ready[6];
	count_write <= fifo_ready[6];
	
	if(fifo_ready[0])
	  led[3:0] <= fpc_data[3:0];
	fpc_read <= fifo_ready[4];
	tpc_write <= fifo_ready[0] && fpc_read;
	tpc_data <= fpc_data;
     end
endmodule
