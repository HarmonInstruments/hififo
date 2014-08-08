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
   output [3:0]     pci_exp_txp,
   output [3:0]     pci_exp_txn,
   input [3:0] 	    pci_exp_rxp,
   input [3:0] 	    pci_exp_rxn,
   input 	    sys_clk_p,
   input 	    sys_clk_n,
   input 	    sys_rst_n,
   // from core
   output 	    clock,
   // FIFOs
   input [7:0] 	    fifo_clock,
   output [7:0]      fifo_reset,
   input [7:0] 	    fifo_rw,
   output [7:0]     fifo_ready,
   output [63:0]    fifo_data_0,
   output [63:0]    fifo_data_1,
   output [63:0]    fifo_data_2,
   output [63:0]    fifo_data_3,
   input [63:0]     fifo_data_4,
   input [63:0]     fifo_data_5,
   input [63:0]     fifo_data_6,
   input [63:0]     fifo_data_7
   );
   
   parameter ENABLE = 8'b00010001;
   parameter NBITS_TAG_LOW = 3;
      
   wire [15:0] 	 pci_id;
   wire 	 pci_reset;
      
   // from RX module
   wire 	rx_rc_valid;
   wire [5:0] 	rx_rc_index;
   wire [7:0] 	rx_rc_tag;
   wire 	rx_rr_valid;
   wire 	rx_wr_valid;
   wire [63:0] 	rx_data;
   wire [5:0] 	rx_address;

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
   wire [(4*NBITS_TAG_LOW)-1:0] rr_mux_tag_low;
   
   wire [3:0] 	tx_wr_valid;
   wire [3:0] 	tx_wr_ready;
   wire [65:0] 	tx_wr_data[0:3];

   wire [7:0] 	r_valid;
   wire [60:0] 	r_addr;
   wire [18:0] 	r_count;
   wire [7:0] 	r_ready;
      
   wire [31:0] 	status[0:7];

   // interrupts
   reg 		interrupt;
   wire 	interrupt_rdy;
   wire [7:0] 	interrupt_status;
   wire [7:0] 	interrupt_individual;
   
   wire [63:0] 	fifo_data[0:7];

   reg [7:0] 	fifo_reset_sysclock;
   
   assign fifo_data_0 = fifo_data[0];
   assign fifo_data_1 = fifo_data[1];
   assign fifo_data_2 = fifo_data[2];
   assign fifo_data_3 = fifo_data[3];
   assign fifo_data[4] = fifo_data_4;
   assign fifo_data[5] = fifo_data_5;
   assign fifo_data[6] = fifo_data_6;
   assign fifo_data[7] = fifo_data_7;

   always @ (posedge clock)
     begin
	interrupt <= (interrupt_individual != 0) | (interrupt & ~interrupt_rdy & ~pci_reset);
	if(pci_reset)
	  fifo_reset_sysclock <= 8'hFF;
	else if(rx_wr_valid)
	  case(rx_address)
	    3: fifo_reset_sysclock <= fifo_reset_sysclock | rx_data[7:0]; // set
	    4: fifo_reset_sysclock <= fifo_reset_sysclock & ~rx_data[7:0]; // clear
	  endcase
	if(rx_rr_valid)
	  case(rx_address)
	    0:  tx_rc_data <= interrupt_status;
	    1:  tx_rc_data <= ENABLE;
	    3:  tx_rc_data <= fifo_reset_sysclock;
	    4:  tx_rc_data <= fifo_reset_sysclock;
	    16: tx_rc_data <= status[0];
	    18: tx_rc_data <= status[1];
	    20: tx_rc_data <= status[2];
	    22: tx_rc_data <= status[3];
	    24: tx_rc_data <= status[4];
	    26: tx_rc_data <= status[5];
	    28: tx_rc_data <= status[6];
	    30: tx_rc_data <= status[7];
	    default: tx_rc_data <= 1'b0;
	  endcase
	tx_rc_done <= rx_rr_valid;
     end
   
   genvar i;
   generate
      for (i = 0; i < 8; i = i+1) begin: fifo
	 // i = 0 to 4: FPC FIFO
	 if((2**i & ENABLE & 8'h0F) != 0) 
	   begin
	      hififo_interrupt #(.NBITS(23)) hififo_interrupt
		(.clock(clock),
		 .reset(pci_reset),
		 .write(rx_wr_valid && (rx_address == (16 + 2*i))),
		 .wdata(rx_data[31:0]),
		 .count(status[i]),
		 .read(rx_rr_valid && (rx_address == 0)),
		 .out(interrupt_individual[i]),
		 .status(interrupt_status[i]));
	      pcie_from_pc_fifo fpc_fifo
		(.clock(clock),
		 .reset(fifo_reset_sysclock[i]),
		 .status(status[i]),
		 .fifo_number(i[1:0]),
		 // read completion
		 .rc_valid(rx_rc_valid),
		 .rc_tag(rx_rc_tag),
		 .rc_index(rx_rc_index),
		 .rc_data(rx_data),
		 // read request
		 .rr_valid(rr_mux_valid[i]),
		 .rr_ready(rr_mux_ready[i]),
		 .rr_tag_low(rr_mux_tag_low[((i+1)*NBITS_TAG_LOW)-1:i*NBITS_TAG_LOW]),    
		 // FIFO
		 .fifo_clock(fifo_clock[i]),
		 .fifo_read(fifo_rw[i] & ~fifo_reset[i]),
		 .fifo_read_data(fifo_data[i]),
		 .fifo_read_valid(fifo_ready[i])
		 );
	   end
	 else if(i<4)
	   begin
	      assign fifo_data[i] = 0;
	      assign rr_mux_valid[i] = 0;
	      assign rr_mux_tag_low[((i+1)*NBITS_TAG_LOW+1)-1:i*NBITS_TAG_LOW] = 0;
	   end
	 // i = 4 to 7: TPC FIFO
	 if((2**i & ENABLE & 8'hF0) != 0) 
	   begin
	      hififo_interrupt #(.NBITS(29)) hififo_interrupt
		(.clock(clock),
		 .reset(pci_reset),
		 .write(rx_wr_valid && (rx_address == (16 + 2*i))),
		 .wdata(rx_data[31:0]),
		 .count(status[i]),
		 .read(rx_rr_valid && (rx_address == 0)),
		 .out(interrupt_individual[i]),
		 .status(interrupt_status[i]));
	      hififo_tpc_fifo tpc_fifo
		(.clock(clock),
		 .reset(fifo_reset_sysclock[i]),
		 .pci_id(pci_id),
		 .status(status[i]),
		 // request unit
		 .r_valid(r_valid[i]),
		 .r_addr(r_addr),
		 .r_count(r_count),
		 .r_ready(r_ready[i]),
		 // write request to TX
		 .wr_valid(tx_wr_valid[i-4]),
		 .wr_ready(tx_wr_ready[i-4]),
		 .wr_data(tx_wr_data[i-4]),
		 // user FIFO
		 .fifo_clock(fifo_clock[i]),
		 .fifo_data(fifo_data[i]),
		 .fifo_write(fifo_rw[i] & ~fifo_reset[i]),
		 .fifo_ready(fifo_ready[i])
		 );
	   end
	 else if(i>4)
	   begin
	      assign tx_wr_data[i-4] = 0;
	      assign tx_wr_valid[i-4] = 0;
	      assign r_ready[i] = 0;
	   end
	 if((2**i & ENABLE) == 0)
	   begin
	      assign status[i] = 0;
	      assign fifo_ready[i] = 0;
	      assign interrupt_individual[i] = 0;
	      assign interrupt_status[i] = 0;
	      assign fifo_reset[i] = 1'b1;
	   end
	 else
	   begin
	      wire reset_sync_out;
	      reg [3:0] reset_count = 0;
	      reg 	reset_out = 1;
	      assign fifo_reset[i] = reset_out;
	      always @ (posedge fifo_clock[i])
		begin
		   reset_out <= reset_count != 15;
		   reset_count <= reset_sync_out ? 4'd0 : reset_count + (reset_count != 15);
		end
	      sync sync(.clock(fifo_clock[i]), .in(fifo_reset_sysclock[i]), .out(reset_sync_out));
	   end
      end
   endgenerate
   
   hififo_request #(.ENABLE(ENABLE)) hififo_request
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
   
   fpc_rr_mux #(.ENABLE(ENABLE), .NBITS_TAG_LOW(NBITS_TAG_LOW)) fpc_rr_mux
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
      .rr_tag_low(rr_mux_tag_low),
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
      .wr_data0(tx_wr_data[0]),
      .wr_data1(tx_wr_data[1]),
      .wr_data2(tx_wr_data[2]),
      .wr_data3(tx_wr_data[3]),
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
      .interrupt_rdy(interrupt_rdy),
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

