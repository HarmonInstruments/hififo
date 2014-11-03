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
   input [15:0]  pci_id,
   // read completion (rc)
   input [31:0]  rc_dw2,
   input [31:0]  rc_data,
   input 	 rc_valid,
   output reg 	 rc_ready,
   // read request (rr)
   input 	 rr_valid,
   output reg 	 rr_ready,
   input [63:0]  rr_addr,
   input [7:0] 	 rr_tag,
   // write request (wr)
   input 	 wr_valid,
   output  	 wr_ready, // pulses wr_count times in request of the next data value
   input [63:0]  wr_data,
   input [63:0]  wr_addr,
   input [4:0] 	 wr_count,
   input 	 wr_last,
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

   reg [2:0] 	     state = 0;
   // FIFO input
   wire 	     fi_ready;
   reg [65:0] 	     fi_data;
   reg 		     fi_valid = 0;
   // wr
   reg [63:0] 	     wr_data_next;
   reg 		     wr_is_32;
   // rr
   wire 	     rr_is_32 = rr_addr[63:32] == 0;
   reg 		     rr_is_32_q;

   assign wr_ready = (state == 5);

   always @(posedge clock)
     begin
	if(reset)
	  state <= 5'd0;
	else
	  case(state)
	    default: state <= ~fi_ready ? 3'd0 :
			      rc_valid ? 3'd1 :
			      rr_valid ? 3'd3 :
			      wr_valid ? 3'd5 : 3'd0;
	    1: state <= 3'd2;
	    2: state <= 3'd0;
	    3: state <= 3'd4;
	    4: state <= 3'd0;
	    5: state <= 3'd6;
	    6: state <= 3'd7;
	    7: state <= ~wr_last ? 3'd7 :
			~fi_ready ? 3'd0 :
			rc_valid ? 3'd1 :
			rr_valid ? 3'd3 :
			wr_valid ? 3'd5 : 3'd0;
	  endcase
	fi_valid <= state != 0;
	case(state)
	  // read completion (rc)
	  1: fi_data <= {2'b00, pci_id, 16'd4, 32'h4A000001}; // always 1 DW
	  2: fi_data <= {2'b01, es(rc_data), rc_dw2};
	  // read request (rr)
	  3: fi_data <= {2'b00, {pci_id, rr_tag[7:0], 8'hFF},
			 {2'd0, ~rr_is_32, 29'd128}};
	  4: fi_data <= {rr_is_32_q, 1'b1, rr_addr[31:0],
			 rr_is_32_q ? rr_addr[31:0] : rr_addr[63:32]};
	  // write request (wr)
	  5: begin
	     wr_is_32 <= wr_addr[63:32] == 0;
	     fi_data <= {2'b00, pci_id, 16'h00FF, 2'b01, wr_addr[63:32] != 0,
			 23'd0, wr_count, 1'b0};
	  end
	  6: begin
	     fi_data <= {2'b00, wr_is_32 ?
			 {es(wr_data[31:0]), wr_addr[31:0]} :
			 {wr_addr[31:0], wr_addr[63:32]}};
	  end
	  7: begin
	     fi_data[65:64] <= wr_last ? {wr_is_32, 1'b1} : 2'b00;
	     fi_data[63:0] <= {2'b00, wr_is_32 ?
			       {es(wr_data[31:0]), es(wr_data_next[63:32])} :
			       {es(wr_data_next[63:32]),
				es(wr_data_next[31:0])
				}
			       };
	  end
	  default: fi_data <= 1'b0;
	endcase
	wr_data_next <= wr_data;
	rr_ready <= (state == 3);
	rr_is_32_q <= rr_is_32;
	rc_ready <= (state == 1);
     end

   fwft_fifo #(.NBITS(66), .FULL_OFFSET(9'h1C0)) tx_fifo
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

module rr_mux4
  (
   input 		clock,
   input 		reset,
   input [3:0] 		rri_valid,
   output [3:0] 	rri_ready,
   input [63:0] 	rri_addr_0,
   input [63:0] 	rri_addr_1,
   input [63:0] 	rri_addr_2,
   input [63:0] 	rri_addr_3,
   input [TAG-1:0] 	rri_tag_0,
   input [TAG-1:0] 	rri_tag_1,
   input [TAG-1:0] 	rri_tag_2,
   input [TAG-1:0] 	rri_tag_3,
   output reg 		rro_valid = 0,
   input 		rro_ready,
   output [63:0] 	rro_addr,
   output reg [TAG-1:0] rro_tag
   );
   parameter TAG = 8;
   parameter AMIN = 0; // lowest address bit to keep
   reg [3:0] 		state = 0;
   reg [63-AMIN:0] 	rro_addr_s;
   assign rro_addr[63:AMIN] = rro_addr_s;
   assign rro_addr[AMIN-1:0] = 0;

   always @ (posedge clock)
     begin
	if(reset)
	  state <= 4'h0;
	else
	  begin
	     casex(state)
	       4'h0: state <= state + (rri_valid[0] ? 4'd1 : 4'd4);
	       4'h4: state <= state + (rri_valid[1] ? 4'd1 : 4'd4);
	       4'h8: state <= state + (rri_valid[2] ? 4'd1 : 4'd4);
	       4'hC: state <= state + (rri_valid[3] ? 4'd1 : 4'd4);
	       4'bxx01: state <= state + rro_ready;
	       default: state <= state + 1'b1;
	     endcase
	  end
	rro_valid <= (state[1:0] == 1) && ~rro_ready;
	case(state[3:2])
	  0: rro_addr_s <= rri_addr_0[63:AMIN];
	  1: rro_addr_s <= rri_addr_1[63:AMIN];
	  2: rro_addr_s <= rri_addr_2[63:AMIN];
	  3: rro_addr_s <= rri_addr_3[63:AMIN];
	endcase
	case(state[3:2])
	  0: rro_tag <= rri_tag_0;
	  1: rro_tag <= rri_tag_1;
	  2: rro_tag <= rri_tag_2;
	  3: rro_tag <= rri_tag_3;
	endcase
     end
   assign rri_ready[0] = state == 2;
   assign rri_ready[1] = state == 6;
   assign rri_ready[2] = state == 10;
   assign rri_ready[3] = state == 14;
endmodule

module wr_mux
  (
   input 	     clock,
   input 	     reset,
   input [3:0] 	     wri_valid,
   output [3:0]      wri_ready,
   input [3:0] 	     wri_last,
   input [63:0]      wri_addr_0,
   input [63:0]      wri_addr_1,
   input [63:0]      wri_addr_2,
   input [63:0]      wri_addr_3,
   input [63:0]      wri_data_0,
   input [63:0]      wri_data_1,
   input [63:0]      wri_data_2,
   input [63:0]      wri_data_3,
   output 	     wro_valid,
   input 	     wro_ready,
   output [63:0]     wro_addr,
   output reg [63:0] wro_data,
   output [4:0]      wro_count,
   output reg 	     wro_last
   );

   reg [3:0] 	     state = 0;
   reg [60:0] 	     wro_addr_s = 0;

   assign wro_addr = {wro_addr_s, 3'd0};
   assign wro_count = 16;
   assign wri_ready[0] = (state == 1) && wro_ready;
   assign wri_ready[1] = (state == 5) && wro_ready;
   assign wri_ready[2] = (state == 9) && wro_ready;
   assign wri_ready[3] = (state == 13) && wro_ready;
   assign wro_valid = (state[1:0] == 1);
   always @ (posedge clock)
     begin
	if(reset)
	  state <= 4'h0;
	else
	  begin
	     casex(state)
	       default: state <= wri_valid[0] ? 4'h1 :
				 wri_valid[1] ? 4'h5 :
				 wri_valid[2] ? 4'h9 :
				 wri_valid[3] ? 4'hD : 4'h0;
	       4'bxx01: state <= state + wro_ready;
	       4'h2: state <= ~wri_last[0] ? state :
			      wri_valid[1] ? 4'h5 :
			      wri_valid[2] ? 4'h9 :
			      wri_valid[3] ? 4'hD :
			      wri_valid[0] ? 4'h1 : 4'h0;
	       4'h6: state <= ~wri_last[1] ? state :
			      wri_valid[0] ? 4'h1 :
			      wri_valid[2] ? 4'h9 :
			      wri_valid[3] ? 4'hD :
			      wri_valid[1] ? 4'h5 : 4'h0;
	       4'hA: state <= ~wri_last[2] ? state :
			      wri_valid[3] ? 4'hD :
			      wri_valid[0] ? 4'h1 :
			      wri_valid[1] ? 4'h5 :
			      wri_valid[2] ? 4'h9 : 4'h0;
	       4'hE: state <= ~wri_last[3] ? state :
			      wri_valid[0] ? 4'h1 :
			      wri_valid[1] ? 4'h5 :
			      wri_valid[2] ? 4'h9 :
			      wri_valid[3] ? 4'hD : 4'h0;
	     endcase
	  end
	case(state[3:2])
	  0: wro_addr_s <= wri_addr_0[63:3];
	  1: wro_addr_s <= wri_addr_1[63:3];
	  2: wro_addr_s <= wri_addr_2[63:3];
	  3: wro_addr_s <= wri_addr_3[63:3];
	endcase
	case(state[3:2])
	  0: wro_data <= wri_data_0;
	  1: wro_data <= wri_data_1;
	  2: wro_data <= wri_data_2;
	  3: wro_data <= wri_data_3;
	endcase
	case(state[3:2])
	  0: wro_last <= wri_last[0];
	  1: wro_last <= wri_last[1];
	  2: wro_last <= wri_last[2];
	  3: wro_last <= wri_last[3];
	endcase
     end
endmodule

module rr_mux
  (
   input 	 clock,
   input 	 reset,
   input [3:0] 	 rri_valid,
   output [3:0]  rri_ready,
   input [63:0]  rri_addr_0,
   input [63:0]  rri_addr_1,
   input [63:0]  rri_addr_2,
   input [63:0]  rri_addr_3,
   input [2:0] 	 rri_tag_0,
   input [2:0] 	 rri_tag_1,
   input [2:0] 	 rri_tag_2,
   input [2:0] 	 rri_tag_3,
   output 	 rro_valid,
   input 	 rro_ready,
   output [63:0] rro_addr,
   output [7:0]  rro_tag
   );

   wire [4:0] 	 rro_tag_raw;
   assign rro_tag = {2'b0, rro_tag_raw[4:3], 1'b0, rro_tag_raw[2:0]};

   rr_mux4 #(.TAG(5), .AMIN(9)) rr_mux4_0
     (.clock(clock),
      .reset(reset),
      .rri_valid(rri_valid[3:0]),
      .rri_ready(rri_ready[3:0]),
      .rri_addr_0(rri_addr_0),
      .rri_addr_1(rri_addr_1),
      .rri_addr_2(rri_addr_2),
      .rri_addr_3(rri_addr_3),
      .rri_tag_0({2'd0,rri_tag_0}),
      .rri_tag_1({2'd1,rri_tag_1}),
      .rri_tag_2({2'd2,rri_tag_2}),
      .rri_tag_3({2'd3,rri_tag_3}),
      .rro_valid(rro_valid),
      .rro_ready(rro_ready),
      .rro_addr(rro_addr),
      .rro_tag(rro_tag_raw)
      );

endmodule

