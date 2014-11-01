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
`define USE_GT_DRP

module hififo_pcie
  (
   // IO pins
   output [NLANES-1:0] 	  pci_exp_txp,
   output [NLANES-1:0] 	  pci_exp_txn,
   input [NLANES-1:0] 	  pci_exp_rxp,
   input [NLANES-1:0] 	  pci_exp_rxn,
   input 		  sys_clk_p,
   input 		  sys_clk_n,
   input 		  sys_rst_n,
   // from core
   output 		  clock,
   `ifdef USE_GT_DRP
   // GT DRP
   input [9*NLANES-1:0]   gt_drp_address,
   input [NLANES-1:0] 	  gt_drp_en,
   input [16*NLANES-1:0]  gt_drp_di,
   output [16*NLANES-1:0] gt_drp_do,
   output [NLANES-1:0] 	  gt_drp_ready,
   input [NLANES-1:0] 	  gt_drp_we,
   input 		  gt_drp_clock,
   `endif
   // FIFOs
   input [7:0] 		  fifo_clock,
   output [7:0] 	  fifo_reset,
   input [7:0] 		  fifo_rw,
   output [7:0] 	  fifo_ready,
   output [63:0] 	  fifo_data_0,
   output [63:0] 	  fifo_data_1,
   output [63:0] 	  fifo_data_2,
   output [63:0] 	  fifo_data_3,
   input [63:0] 	  fifo_data_4,
   input [63:0] 	  fifo_data_5,
   input [63:0] 	  fifo_data_6,
   input [63:0] 	  fifo_data_7
   );

   parameter NLANES = 4;
   parameter ENABLE = 8'b00010001;

   wire [15:0] 	 pci_id;
   wire 	 pci_reset;

   // from RX module
   wire 	 rx_rc_valid;
   wire [5:0] 	 rx_rc_index;
   wire [7:0] 	 rx_rc_tag;
   wire 	 rx_wr_valid;
   wire 	 rx_rr_valid;
   wire [23:0] 	 rx_rr_rid_tag;
   wire [7:0] 	 rx_rr_addr;
   reg 		 rx_rr_ready;
   wire [63:0] 	 rx_data;
   wire [5:0] 	 rx_address;

   // read completion request to TX module
   wire [31:0] 	 tx_rc_dw2 = {rx_rr_rid_tag, 1'b0, rx_rr_addr[4:0], 2'd0};
   reg [31:0] 	 tx_rc_data;
   reg 		 tx_rc_valid = 0;
   wire 	 tx_rc_ready;

   wire [3:0] 	 mux_rr_valid, mux_rr_ready;
   wire [63:0] 	 mux_rr_addr [0:3];
   wire [2:0] 	 mux_rr_tag [0:3];

   wire [3:0] 	 mux_wr_valid, mux_wr_ready;
   wire [63:0] 	 mux_wr_data[0:3];
   wire [63:0] 	 mux_wr_addr[0:3];
   wire [4:0] 	 mux_wr_count[0:3];
   wire [3:0] 	 mux_wr_last;

   wire [31:0] 	 status[0:7];

   // interrupts
   reg 		 interrupt;
   wire 	 interrupt_rdy;
   reg [7:0] 	 interrupt_status = 0;
   wire [7:0] 	 interrupt_individual;

   wire [63:0] 	 fifo_data[0:7];

   reg [7:0] 	 fifo_reset_sysclock;

   assign fifo_data_0 = fifo_data[0];
   assign fifo_data_1 = fifo_data[1];
   assign fifo_data_2 = fifo_data[2];
   assign fifo_data_3 = fifo_data[3];
   assign fifo_data[4] = fifo_data_4;
   assign fifo_data[5] = fifo_data_5;
   assign fifo_data[6] = fifo_data_6;
   assign fifo_data[7] = fifo_data_7;

   reg [1:0] 	 read = 0;
   reg 		 read_in_progress = 0;

   always @ (posedge clock)
     begin
	interrupt <= (interrupt_individual != 0)
	  | (interrupt & ~interrupt_rdy & ~pci_reset);
	if(pci_reset | (read[1] && (rx_rr_addr[4:1] == 0)))
	  interrupt_status <= 1'b0;
	else
	  interrupt_status <= interrupt_status | interrupt_individual;
	if(pci_reset)
	  fifo_reset_sysclock <= 8'hFF;
	else if(rx_wr_valid)
	  case(rx_address)
	    // set
	    3: fifo_reset_sysclock <= fifo_reset_sysclock | rx_data[7:0];
	    // clear
	    4: fifo_reset_sysclock <= fifo_reset_sysclock & ~rx_data[7:0];
	  endcase
	if(tx_rc_ready)
	  read_in_progress <= 1'b0;
	else if(rx_rr_valid & ~rx_rr_ready)
	  read_in_progress <= 1'b1;
	rx_rr_ready <= tx_rc_ready;
	read[0] <= rx_rr_valid & ~read_in_progress & ~rx_rr_ready;
	read[1] <= read[0];
	if(tx_rc_ready)
	  tx_rc_valid <= 1'b0;
	else if(read[1])
	  tx_rc_valid <= 1'b1;

	if(read[1])
	  case(rx_rr_addr[4:1])
	    0:  tx_rc_data <= interrupt_status;
	    1:  tx_rc_data <= ENABLE;
	    3:  tx_rc_data <= fifo_reset_sysclock;
	    4:  tx_rc_data <= fifo_reset_sysclock;
	    8:  tx_rc_data <= status[0];
	    9:  tx_rc_data <= status[1];
	    10: tx_rc_data <= status[2];
	    11: tx_rc_data <= status[3];
	    12: tx_rc_data <= status[4];
	    13: tx_rc_data <= status[5];
	    14: tx_rc_data <= status[6];
	    15: tx_rc_data <= status[7];
	    default: tx_rc_data <= 1'b0;
	  endcase
     end

   genvar i;
   generate
      for (i = 0; i < 8; i = i+1) begin: fifo
	 // i = 0 to 4: FPC FIFO
	 if((2**i & ENABLE & 8'h0F) != 0)
	   begin
	      pcie_from_pc_fifo fpc_fifo
		(.clock(clock),
		 .reset(fifo_reset_sysclock[i]),
		 .status(status[i]),
		 .interrupt(interrupt_individual[i]),
		 .fifo_number(i[1:0]),
		 // read completion
		 .rc_valid(rx_rc_valid),
		 .rc_tag(rx_rc_tag),
		 .rc_index(rx_rc_index),
		 .pio_wvalid(rx_wr_valid && (rx_address == 8+i)),
		 .rx_data(rx_data),
		 // read request
		 .rr_valid(mux_rr_valid[i]),
		 .rr_ready(mux_rr_ready[i]),
		 .rr_addr(mux_rr_addr[i]),
		 .rr_tag(mux_rr_tag[i]),
		 // FIFO
		 .fifo_clock(fifo_clock[i]),
		 .fifo_read(fifo_rw[i] & ~fifo_reset[i]),
		 .fifo_read_data(fifo_data[i]),
		 .fifo_read_valid(fifo_ready[i])
		 );
	   end
	 else if(i<4)
	   begin
	      assign mux_rr_tag[i] = 0;
	      assign fifo_data[i] = 0;
	      assign mux_rr_valid[i] = 0;
	      assign mux_rr_addr[i] = 0;
	   end
	 // i = 4 to 7: TPC FIFO
	 if((2**i & ENABLE & 8'hF0) != 0)
	   begin
	      hififo_tpc_fifo tpc_fifo
		(.clock(clock),
		 .reset(fifo_reset_sysclock[i]),
		 .status(status[i]),
		 .interrupt(interrupt_individual[i]),
		 // write data
		 .rx_data(rx_data),
		 .rx_data_valid(rx_wr_valid && (rx_address == 8+i)),
		 // write request to TX
		 .wr_valid(mux_wr_valid[i-4]),
		 .wr_ready(mux_wr_ready[i-4]),
		 .wr_data(mux_wr_data[i-4]),
		 .wr_addr(mux_wr_addr[i-4]),
		 .wr_count(mux_wr_count[i-4]),
		 .wr_last(mux_wr_last[i-4]),
		 // user FIFO
		 .fifo_clock(fifo_clock[i]),
		 .fifo_data(fifo_data[i]),
		 .fifo_write(fifo_rw[i] & ~fifo_reset[i]),
		 .fifo_ready(fifo_ready[i])
		 );
	   end
	 else if(i>4)
	   begin
	      assign mux_wr_last[i-4] = 0;
	      assign mux_wr_data[i-4] = 0;
	      assign mux_wr_count[i-4] = 0;
	      assign mux_wr_addr[i-4] = 0;
	      assign mux_wr_valid[i-4] = 0;
	   end
	 if((2**i & ENABLE) == 0)
	   begin
	      assign status[i] = 0;
	      assign fifo_ready[i] = 0;
	      assign interrupt_individual[i] = 0;
	      assign fifo_reset[i] = 1'b1;
	   end
	 else
	   begin
	      wire reset_sync_out;

	      sync sync(
			.clock(fifo_clock[i]),
			.in(fifo_reset_sysclock[i]),
			.out(reset_sync_out)
			);

	      pulse_stretch #(.NB(4)) stretch_reset
		(
		 .clock(fifo_clock[i]),
		 .in(reset_sync_out),
		 .out(fifo_reset[i])
		 );
	   end
      end
   endgenerate

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
      .completion_valid(rx_rc_valid),
      .completion_index(rx_rc_index),
      .completion_tag(rx_rc_tag),
      .data(rx_data),
      .address(rx_address),
      .rr_valid(rx_rr_valid),
      .rr_ready(rx_rr_ready),
      .rr_rid_tag(rx_rr_rid_tag),
      .rr_addr(rx_rr_addr),
      // AXI stream from PCIE core
      .tvalid(m_axis_rx_tvalid),
      .tlast(m_axis_rx_tlast),
      .tdata(m_axis_rx_tdata)
      );

   wire 	 tx_rr_valid;
   wire 	 tx_rr_ready;
   wire [63:0] 	 tx_rr_addr;
   wire [7:0] 	 tx_rr_tag;

   rr_mux rr_mux
     (.clock(clock),
      .reset(pci_reset),
      .rri_valid(mux_rr_valid[3:0]),
      .rri_ready(mux_rr_ready[3:0]),
      .rri_addr_0(mux_rr_addr[0]),
      .rri_addr_1(mux_rr_addr[1]),
      .rri_addr_2(mux_rr_addr[2]),
      .rri_addr_3(mux_rr_addr[3]),
      .rri_tag_0(mux_rr_tag[0]),
      .rri_tag_1(mux_rr_tag[1]),
      .rri_tag_2(mux_rr_tag[2]),
      .rri_tag_3(mux_rr_tag[3]),
      .rro_valid(tx_rr_valid),
      .rro_ready(tx_rr_ready),
      .rro_addr(tx_rr_addr),
      .rro_tag(tx_rr_tag));

   wire  	 tx_wr_valid, tx_wr_ready, tx_wr_last;
   wire [63:0] 	 tx_wr_data;
   wire [63:0] 	 tx_wr_addr;
   wire [4:0] 	 tx_wr_count;

   wr_mux wr_mux
     (.clock(clock),
      .reset(pci_reset),
      .wri_valid(mux_wr_valid),
      .wri_ready(mux_wr_ready),
      .wri_last(mux_wr_last),
      .wri_addr_0(mux_wr_addr[0]),
      .wri_addr_1(mux_wr_addr[1]),
      .wri_addr_2(mux_wr_addr[2]),
      .wri_addr_3(mux_wr_addr[3]),
      .wri_data_0(mux_wr_data[0]),
      .wri_data_1(mux_wr_data[1]),
      .wri_data_2(mux_wr_data[2]),
      .wri_data_3(mux_wr_data[3]),
      .wro_valid(tx_wr_valid),
      .wro_ready(tx_wr_ready),
      .wro_addr(tx_wr_addr),
      .wro_data(tx_wr_data),
      .wro_last(tx_wr_last),
      .wro_count(tx_wr_count)
     );

   pcie_tx tx
     (.clock(clock),
      .reset(pci_reset),
      .pci_id(pci_id),
      // read completion (rc)
      .rc_valid(tx_rc_valid),
      .rc_ready(tx_rc_ready),
      .rc_dw2(tx_rc_dw2),
      .rc_data(tx_rc_data),
      // read request (rr)
      .rr_valid(tx_rr_valid),
      .rr_ready(tx_rr_ready),
      .rr_addr(tx_rr_addr),
      .rr_tag(tx_rr_tag),
      // write request (wr)
      .wr_valid(tx_wr_valid),
      .wr_ready(tx_wr_ready),
      .wr_data(tx_wr_data),
      .wr_addr(tx_wr_addr),
      .wr_count(tx_wr_count),
      .wr_last(tx_wr_last),
      // AXI stream to PCI Express core
      .tx_tready(s_axis_tx_tready),
      .tx_tdata(s_axis_tx_tdata),
      .tx_1dw(s_axis_tx_1dw),
      .tx_tlast(s_axis_tx_tlast),
      .tx_tvalid(s_axis_tx_tvalid)
   );

   pcie_core_wrap #(.NLANES(NLANES)) pcie_core_wrap
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
      .interrupt_rdy(interrupt_rdy),
      .pci_reset(pci_reset),
   `ifdef USE_GT_DRP
      .gt_drp_address(gt_drp_address),
      .gt_drp_en(gt_drp_en),
      .gt_drp_di(gt_drp_di),
      .gt_drp_do(gt_drp_do),
      .gt_drp_ready(gt_drp_ready),
      .gt_drp_we(gt_drp_we),
      .gt_drp_clock(gt_drp_clock),
   `else
      .gt_drp_address(1'b0),
      .gt_drp_en(1'b0),
      .gt_drp_di(1'b0),
      .gt_drp_do(),
      .gt_drp_ready(),
      .gt_drp_we(1'b0),
      .gt_drp_clock(1'b0),
   `endif
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

