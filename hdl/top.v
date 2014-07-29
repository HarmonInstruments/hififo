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

   wire 	    pio_write_valid;
   wire [63:0] 	    pio_write_data;
   wire [12:0] 	    pio_address;
   
   reg [63:0] 	    tpc_data = 0;
   reg 		    tpc_write = 0;
   wire 	    tpc_ready;
   
   wire [63:0] 	    fpc_data;
   reg 		    fpc_read = 1'b0;
   wire 	    fpc_valid;
   
   wire 	    pci_reset;
   wire [15:0] 	    pci_id;
   // to core
   wire 	    interrupt;
   // AXI from core
   wire 	    s_axis_tx_tready;
   wire [63:0] 	    s_axis_tx_tdata;
   wire  	    s_axis_tx_1dw;
   wire 	    s_axis_tx_tlast;
   wire 	    s_axis_tx_tvalid;
   // AXI from core
   wire 	    m_axis_rx_tvalid;
   wire 	    m_axis_rx_tlast;
   wire [63:0] 	    m_axis_rx_tdata;
   
   hififo_pcie hififo
     (.clock(clock),
      .pci_reset(pci_reset),
      .pci_id(pci_id),
      .interrupt_out(interrupt),
      .s_axis_tx_tready(s_axis_tx_tready),
      .s_axis_tx_tdata(s_axis_tx_tdata),
      .s_axis_tx_1dw(s_axis_tx_1dw),
      .s_axis_tx_tlast(s_axis_tx_tlast),
      .s_axis_tx_tvalid(s_axis_tx_tvalid),
      .m_axis_rx_tvalid(m_axis_rx_tvalid),
      .m_axis_rx_tlast(m_axis_rx_tlast),
      .m_axis_rx_tdata(m_axis_rx_tdata),
      .pio_write_valid(pio_write_valid),
      .pio_write_data(pio_write_data),
      .pio_address(pio_address),
      .fifo_clock(clock),
      .tpc0_reset(),
      .tpc0_data(tpc_data),
      .tpc0_write(tpc_write),
      .tpc0_ready(tpc_ready),
      .fpc0_reset(),
      .fpc0_data(fpc_data),
      .fpc0_read(fpc_read),
      .fpc0_valid(fpc_valid)
      );
   
   reg 		    use_count = 1;
      
   always @ (posedge clock)
     begin
	if(fpc_valid)
	  led[3:0] <= fpc_data[3:0];
	if(pio_write_valid && (pio_address == 15))
	  use_count <= pio_write_data[0];
	
	if(~use_count)
	  begin
	     fpc_read <= 1'b1;
	     tpc_write <= fpc_valid;
	     tpc_data <= fpc_data;
	  end
	else if(tpc_ready)
	  begin
	     fpc_read <= 1'b0;
	     tpc_write <= 1'b1;
	     tpc_data <= tpc_data + 1'b1;
	  end
	else
	  begin
	     fpc_read <= 1'b0;
	     tpc_write <= 1'b0;
	  end
     end
   pcie_core_wrap pcie_core_wrap
     (.pci_exp_txp(pcie_txp),
      .pci_exp_txn(pcie_txn),
      .pci_exp_rxp(pcie_rxp),
      .pci_exp_rxn(pcie_rxn),
      .sys_clk_p(pcie_refclk_p),
      .sys_clk_n(pcie_refclk_n),
      .sys_rst_n(pcie_rst_n),
      .clock(clock),
      .pci_id(pci_id),
      .interrupt(interrupt),
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
