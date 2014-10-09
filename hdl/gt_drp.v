/*
 * PCI Express to FIFO - Xilinx ADC interface
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
 *
 * din[15:0] DRP data
 * din[24:16] DRP address
 * din[31] DRP write enable
 *
 * dout[15:0] DRP data
 * dout[16] busy
 *
 */

`timescale 1ns / 1ps

module gt_drp
  (
   input 	     clock,
   input 	     write,
   input [63:0]      din,
   output reg [16:0] dout = 0,
   // common clock from gt_drp_clock
   input [2:0] 	     clkdiv,
   // GT DRP
   output reg [8:0]  drp_address,
   output reg 	     drp_en = 0,
   output reg [15:0] drp_di,
   input [15:0]      drp_do,
   input 	     drp_ready,
   output reg 	     drp_we
   );

   reg [1:0] 	    state = 0;

   always @ (posedge clock)
     begin
	dout[16] <= state != 0;
	state <= write ? 2'd1 :
		 (state == 1) && (clkdiv == 0) ? 2'd2 :
		 (clkdiv == 0) && drp_ready ? 2'd0 :
		 (state == 2) && (clkdiv == 0) ? 2'd3 :
		 state;
	drp_en <= (state == 1) && (clkdiv == 0) ? 1'b1 :
		  (state == 2) && (clkdiv == 0) ? 1'b0 :
		  drp_en;
	if(write)
	  begin
	     drp_di <= din[15:0];
	     drp_address <= din[24:16];
	     drp_we <= din[31];
	  end
	if((clkdiv == 0) && drp_ready)
	  dout[15:0] <= drp_do;
     end
endmodule

module gt_drp_clock (input clock, output drpclock, output reg [2:0] clkdiv = 0);
   always @ (posedge clock)
     clkdiv <= clkdiv + 1'b1;
   assign drpclock = clkdiv[2];
endmodule