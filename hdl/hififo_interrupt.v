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
   // PIO
   input 	pio_wvalid,
   input [63:0] pio_wdata,
   input [10:0] pio_addr,
   //
   output reg 	interrupt,
   input 	interrupt_rdy,
   output [3:0] interrupt_num,
   input [2:0] 	interrupts_enabled
   );
   parameter enables = 8'b00010001;
   parameter LOWBIT_FPC = 9;
   parameter LOWBIT_TPC = 3;
   
   genvar 	i;
   generate
      for (i = 0; i < 8; i = i+1) begin: iproc
	 if((2**i & enables) != 0)
	   begin
	      if(i<4)
		begin
		   wire [31:0]           status;
		   reg [31-LOWBIT_FPC:0] matchval;
		   wire 		 match = matchval == status[31:LOWBIT_FPC];
		   always @ (posedge clock)
		     matchval <= reset ? 1'b0 : (pio_wvalid && (pio_addr == 32 + 4*i)) ? pio_wdata[31:LOWBIT_FPC] : matchval;
		end
	      else
		begin
		   wire [31:0]           status;
		   reg [31-LOWBIT_TPC:0] matchval;
		   wire 		 match = matchval == status[31:LOWBIT_TPC];
		   always @ (posedge clock)
		     matchval <= reset ? 1'b0 : (pio_wvalid && (pio_addr == 32 + 4*i)) ? pio_wdata[31:LOWBIT_TPC] : matchval;
		end
	      always @ (posedge clock)
		begin
		end
	   end
	 else
	   begin
	   end
      end
   endgenerate
   
   always @ (posedge clock)
     begin
	if(reset)
	  interrupt <= 1'b0;
	
     end
   
endmodule