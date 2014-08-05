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

module pcie_tx
  (
   input 	 clock,
   input 	 reset,
   input [15:0]  pcie_id,
   // read completion (rc)
   input 	 rc_done,
   input [31:0]  rc_dw2,
   input [31:0]  rc_data,
   // read request (rr)
   input 	 rr_valid,
   input [63:0]  rr_addr,
   input [7:0] 	 rr_tag,
   output 	 rr_ready,
   // write request (wr)
   input [3:0] 	 wr_valid,
   output [3:0]  wr_ready, // pulses 16 times in request of the next data value
   input [65:0]  wr_data0,
   input [65:0]  wr_data1,
   input [65:0]  wr_data2,
   input [65:0]  wr_data3,
   // AXI stream to PCI Express core
   input 	 tx_tready,
   output [63:0] tx_tdata,
   output 	 tx_1dw,
   output 	 tx_tlast,
   output 	 tx_tvalid
   );

   function [31:0] es; // endian swap
      input [31:0]   x;
      es = {x[7:0], x[15:8], x[23:16], x[31:24]};
   endfunction

   reg 		     rc_valid;
   wire 	     rc_ready;
   reg 		     rc_last;
   
   reg [2:0] 	     state = 0;
   wire 	     rr_is_32 = rr_addr[63:32] == 0;
   assign rc_ready = rc_last;
   assign rr_ready = (state == 3);
   assign wr_ready[0] = (state == 4);
   assign wr_ready[1] = (state == 5);
   assign wr_ready[2] = (state == 6);
   assign wr_ready[3] = (state == 7);
   wire 	     fi_ready;
   reg [65:0] 	     fi_data;
   reg 		     fi_valid = 0;
   wire [7:0] 	     valid;
   assign valid[0] = 0;
   assign valid[1] = rc_valid;
   assign valid[2] = rr_valid;
   assign valid[3] = 0;
   assign valid[7:4] = wr_valid;

   always @(posedge clock)
     begin
	if(reset)
	  rc_valid <= 1'b0;
	else if(rc_done)
	  rc_valid <= 1'b1;
	else if(rc_ready)
	  rc_valid <= 1'b0;
	if(reset)
	  state <= 5'd0;
	else
	  case(state)
	    0: state <= ~fi_ready ? 3'd0 :
			valid[1] ? 3'd1 :
			valid[2] ? 3'd2 :
			valid[4] ? 3'd4 :
			valid[5] ? 3'd5 :
			valid[6] ? 3'd6 :
			valid[7] ? 3'd7 : 3'd0;
	    1: state <= ~rc_last ? state :
			~fi_ready ? 3'd0 :
			valid[2] ? 3'd2 :
			valid[4] ? 3'd4 :
			valid[5] ? 3'd5 :
			valid[6] ? 3'd6 :
			valid[7] ? 3'd7 : 3'd0;
	    2: state <= 3'd3;
	    3: state <= ~fi_ready ? 3'd0 :
			valid[1] ? 3'd1 :
			valid[4] ? 3'd4 :
			valid[5] ? 3'd5 :
			valid[6] ? 3'd6 :
			valid[7] ? 3'd7 : 3'd0;
	    4: state <= ~wr_data0[64] ? state : 
			~fi_ready ? 3'd0 :
			valid[1] ? 3'd1 :
			valid[2] ? 3'd2 :
			valid[5] ? 3'd5 :
			valid[6] ? 3'd6 :
			valid[7] ? 3'd7 : 3'd0;
	    5: state <= ~wr_data1[64] ? state : 
			~fi_ready ? 3'd0 :
			valid[1] ? 3'd1 :
			valid[2] ? 3'd2 :
			valid[4] ? 3'd4 :
			valid[6] ? 3'd6 :
			valid[7] ? 3'd7 : 3'd0;
	    6: state <= ~wr_data2[64] ? state : 
			~fi_ready ? 3'd0 :
			valid[1] ? 3'd1 :
			valid[2] ? 3'd2 :
			valid[4] ? 3'd4 :
			valid[5] ? 3'd5 :
			valid[7] ? 3'd7 : 3'd0;
	    7: state <= ~wr_data3[64] ? state : 
			~fi_ready ? 3'd0 :
			valid[1] ? 3'd1 :
			valid[2] ? 3'd2 :
			valid[4] ? 3'd4 :
			valid[5] ? 3'd5 :
			valid[6] ? 3'd6 : 3'd0;
	  endcase
	fi_valid <= state != 0;
	rc_last <= (state == 1) && ~rc_last;
	case(state)
	  // read completion (rc)
	  1: fi_data <= {1'b0, rc_last, rc_last ? {es(rc_data), rc_dw2} : {pcie_id, 16'd8, 32'h4A000001}}; // always 1 DW
	  // read request (rr)
	  2: fi_data <= {2'b00, pcie_id, rr_tag[7:0], 8'hFF, 2'd0, ~rr_is_32, 29'd128}; // always 128 DW
	  3: fi_data <= rr_is_32 ? {2'b11, rr_addr[31:0], rr_addr[31:0]} : {2'b01, rr_addr[31:0], rr_addr[63:32]};
	  // write request (wr)
	  4: fi_data <= wr_data0;
	  5: fi_data <= wr_data1;
	  6: fi_data <= wr_data2;
	  7: fi_data <= wr_data3;
	  default: fi_data <= 1'b0;
	endcase
     end

   fwft_fifo #(.NBITS(66)) tx_fifo
     (
      .reset(reset),
      .i_clock(clock),
      .i_data(fi_data),
      .i_valid(fi_valid),
      .i_ready(fi_ready),
      .o_clock(clock),
      .o_read(tx_tready & tx_tvalid),
      .o_data({tx_1dw, tx_tlast, tx_tdata}),
      .o_valid(tx_tvalid),
      .o_almost_empty()
      );

endmodule
