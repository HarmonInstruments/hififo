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

module hififo_interrupt
  (
   input 	clock,
   input 	reset,
   input 	write,
   input [31:0] wdata,
   input [31:0] count,
   input 	read,
   output reg 	out = 0, // high one cycle after match
   output reg 	status = 0 // high until status read
   );
   parameter NBITS = 32;
   reg [NBITS-1:0] matchval = 0;
   wire 	   match = matchval == count[31:32-NBITS];
   reg 		   match_prev;
   reg 		   enable = 0;
   
   always @ (posedge clock)
     begin
	if(reset | out)
	  enable <= 1'b0;
	else if(write)
	  enable <= ~wdata[0];
	if(write)
	  matchval <= wdata[31:32-NBITS];
	match_prev <= match;
	out <= enable & match & ~match_prev;
	status <= out | (status & ~read & ~reset);
     end
endmodule