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
   input 		       clock,
   input 		       reset,
   input [15:0] 	       pci_id,
   // from request unit
   input [3:0] 		       r_valid,
   input [3:0] 		       r_abort,
   input [60:0] 	       r_addr, // 8 bytes
   input [18:0] 	       r_count, // 8 bytes
   output [3:0] 	       r_ready,
   // read request in
   input [3:0] 		       rr_valid,
   output [3:0] 	       rr_ready,
   input [4*NBITS_TAG_LOW-1:0] rr_tag_low,
   // rr request multiplexed
   output 		       rrm_valid,
   output [65:0] 	       rrm_data,
   input 		       rrm_ready
   );

   parameter ENABLE = 8'b00010001;
   parameter NBITS_TAG_LOW = 3;
   
   reg [3:0] 	     state;
   wire [54:0] 	     rr_addr[0:3];
   wire [3:0] 	     both_valid;
   reg [2:0] 	     rrm_tag_low;
   reg [54:0] 	     rrm_addr;
   assign 	     rrm_valid = state[1:0] == 3;
   
   genvar 	     i;
   generate
      for (i = 0; i < 4; i = i+1) begin: block_fill
	 if((2**i & ENABLE) != 0)
	   begin
	      reg req_valid;
	      reg [54:0] addr;
	      reg [12:0] count;
	      assign both_valid[i] = req_valid & rr_valid[i];
	      assign r_ready[i] = ~req_valid;
	      assign rr_ready[i] = req_valid && (state == 4*i+3);
	      assign rr_addr[i] = addr;
	      always @(posedge clock)
		begin
		   addr <= r_valid[i] ? r_addr[60:6] : addr + (state == 2 + 4*i);
		   count <= (reset | r_abort[i]) ? 1'b0 :
			    r_valid[i] ? r_count[18:6] : count - (state == 2 + 4*i);
		   req_valid <= count != 0;
		end
	   end
	 else
	   begin
	      assign r_ready[i] = 0;
	      assign both_valid[i] = 0;
	      assign rr_ready[i] = 0;
	      assign rr_addr[i] = 0;
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
	       0: rrm_tag_low <= rr_tag_low[1 * NBITS_TAG_LOW - 1: 0 * NBITS_TAG_LOW];
	       1: rrm_tag_low <= rr_tag_low[2 * NBITS_TAG_LOW - 1: 1 * NBITS_TAG_LOW];
	       2: rrm_tag_low <= rr_tag_low[3 * NBITS_TAG_LOW - 1: 2 * NBITS_TAG_LOW];
	       3: rrm_tag_low <= rr_tag_low[4 * NBITS_TAG_LOW - 1: 3 * NBITS_TAG_LOW];
	     endcase
	     rrm_addr <= rr_addr[state[3:2]];
	  end
     end
   
   wire [7:0]  rrm_tag = {2'b0, state[3:2], 1'b0, rrm_tag_low};
   wire [63:0] rrm_addr_s = {rrm_addr, 9'd0};
   wire        rr_is_32 = rrm_addr_s[63:32] == 0;
   wire [31:0] rr_dw0 = {2'd0, ~rr_is_32, 29'd128};
   wire [31:0] rr_dw1 = {pci_id, rrm_tag[7:0], 8'hFF};
   wire [31:0] rr_dw2 = rr_is_32 ? rrm_addr_s[31:0] : rrm_addr_s[63:32];
   wire [31:0] rr_dw3 = rrm_addr_s[31:0];
   wire        rr_last = rrm_ready;
   reg [65:0]  rrm_next = 0;
   assign rrm_data = rrm_next;
   always @ (posedge clock)
     rrm_next <= {rr_last & rr_is_32, rr_last, rr_last ? {rr_dw3, rr_dw2} : {rr_dw1, rr_dw0}};
endmodule
