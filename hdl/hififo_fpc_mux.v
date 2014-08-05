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

module fpc_rr_mux
  (
   input 	     clock,
   input 	     reset,
   // from request unit
   input [3:0] 	     r_valid,
   input [60:0]      r_addr, // 8 bytes
   input [18:0]      r_count, // 8 bytes
   output [3:0]      r_ready,
   // read request in
   input [3:0] 	     rr_valid,
   output [3:0]      rr_ready,
   input [2:0] 	     rr0_tag_low,
   input [2:0] 	     rr1_tag_low,
   input [2:0] 	     rr2_tag_low,
   input [2:0] 	     rr3_tag_low,
   // rr request multiplexed
   output  	     rrm_valid,
   output reg [54:0] rrm_addr,
   output [7:0]      rrm_tag,
   input 	     rrm_ready
   );

   reg [3:0] 	     state;
   wire [3:0] 	     o_req_valid;
   wire [54:0] 	     rr_addr[0:3];
   wire [3:0] 	     both_valid;
   reg [2:0] 	     rrm_tag_low;
   assign rrm_tag = {2'b0, state[3:2], 1'b0, rrm_tag_low};
   assign rrm_valid = state[1:0] == 3;
   
   genvar 	     i;
   generate
      for (i = 0; i < 4; i = i+1) begin: block_fill
	 if(i>0) 
	   begin
	      assign r_ready[i] = 1'b0;
	      assign both_valid[i] = 1'b0;
	      assign rr_ready[i] = 1'b0;
	      assign rr_addr[i] = 1'b0;
	   end
	 else
	   begin
	      wire req_valid;
	      assign both_valid[i] = req_valid & rr_valid[i];
	      assign r_ready[i] = ~req_valid;
	      assign rr_ready[i] = req_valid && (state == 2*i+1);
	      request_count rcount
		(.clock(clock),
		 .reset(reset), 
		 .i_valid(r_valid[i]),
		 .i_addr(r_addr[60:6]),
		 .i_count(r_count[18:6]), 
		 .o_ready(state == 2 + 4*i),
		 .o_valid(req_valid),
		 .o_addr(rr_addr[i]));
	   end
      end
   endgenerate
   
   always @ (posedge clock)
     begin
	if(reset)
	  state <= 1'b0;
	else
	  case(state[1:0])
	    0: state <= both_valid[state[3:2]] ? state + 1'b1 : state + 4'd4;
	    1: state <= state + 1'b1;
	    2: state <= state + 1'b1;
	    3: state <= state + rrm_ready;
	  endcase
	if(state[1:0] == 0)
	  begin
	     case(state[3:2])
	       0: rrm_tag_low <= rr0_tag_low;
	       1: rrm_tag_low <= rr1_tag_low;
	       2: rrm_tag_low <= rr2_tag_low;
	       3: rrm_tag_low <= rr3_tag_low;
	     endcase
	     rrm_addr <= rr_addr[state[3:2]];
	  end
   end
endmodule

module request_count
  (
   input 	     clock,
   input 	     reset,
   input 	     i_valid,
   input [54:0]      i_addr,
   input [12:0]      i_count,
   input 	     o_ready,
   output reg 	     o_valid,
   output reg [54:0] o_addr);

   reg [12:0] 	     o_count;
      
   always @(posedge clock)
     begin
	if(reset)
	  begin
	     o_addr <= 1'b0;
	     o_count <= 1'b0;
	  end
	else if(i_valid)
	  begin
	     o_addr <= i_addr;
	     o_count <= i_count;
	  end
	else if(o_ready)
	  begin
	     o_addr <= o_addr + 1'd1;
	     o_count <= o_count - 1'b1;
	  end
	o_valid <= o_count != 0;
     end
endmodule