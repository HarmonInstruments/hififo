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

module block_ram
  (
   input 		  clock,
   input [DBITS-1:0] 	  w_data,
   input 		  w_valid,
   input [ABITS-1:0] 	  w_addr,
   output reg [DBITS-1:0] r_data,
   input [ABITS-1:0] 	  r_addr
   );
   parameter ABITS = 9;
   parameter DBITS = 64;
   reg [DBITS-1:0] 	  bram[0:2**ABITS-1];
   reg [DBITS-1:0] 	  bram_oreg;
   always @ (posedge clock)
     begin
	if(w_valid)
	  bram[w_addr] <= w_data;
	bram_oreg <= bram[r_addr];
	r_data <= bram_oreg;
     end
endmodule
