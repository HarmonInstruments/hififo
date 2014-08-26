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

module hififo_fetch_descriptor
  (
   input 	 clock,
   input 	 reset,
   output 	 ready,
   input 	 write_fifo,
   input 	 write_desc_addr,
   input [63:0]  wdata,
   input 	 rc_last,
   output 	 rr_valid, // RR0 is descriptor fetch
   output [63:0] rr_addr,
   input 	 rr_ready,
   output [63:0] desc_data,
   output 	 desc_ready,
   input 	 desc_read
   );
   
   reg [1:0] 	 state;
   reg [54:0] 	 next_desc_addr;
  
   wire 	 fifo_ready;
      
   assign rr_addr = {next_desc_addr, 9'd0};
   assign rr_valid = (state == 2);
   assign ready = (state == 0);
      
   always @ (posedge clock)
     begin
	if(reset)
	  state <= 1'b0;
	else
	  begin
	     case(state)
	       // idle - nothing requested, next_desc_addr not valid
	       0: state <= write_desc_addr;
	       // have the descriptor addr, wait for FIFO
	       1: state <= state + fifo_ready;
	       // read request completed
	       2: state <= state + rr_ready;
	       // wait for it to finish
	       3: begin
		  if(rc_last && write_desc_addr)
		    state <= 1'b1;
		  else if(rc_last)
		    state <= 1'b0;
	       end
	     endcase
	  end
      	if(write_desc_addr)
	  next_desc_addr <= wdata[63:9];
     end

   fwft_fifo #(.NBITS(64)) fifo
     (
      .reset(reset),
      .i_clock(clock),
      .i_data(wdata),
      .i_valid(write_fifo),
      .i_ready(fifo_ready),
      .o_clock(clock),
      .o_read(desc_read),
      .o_data(desc_data),
      .o_valid(desc_ready),
      .o_almost_empty()
      );
endmodule