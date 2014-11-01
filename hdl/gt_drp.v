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
   input 	     drp_clock,
   input 	     write,
   input [63:0]      din,
   output reg [16:0] dout = 0,
   // GT DRP
   output reg [8:0]  drp_address,
   output 	     drp_en,
   output reg [15:0] drp_di,
   input [15:0]      drp_do,
   input 	     drp_ready,
   output reg 	     drp_we
   );

   reg [3:0] 	    state = 0;
   reg 		    drp_en_clock = 0;

   wire 	    drp_ready_sync;

   sync sync_ready (.clock(clock), .in(drp_ready), .out(drp_ready_sync));

   sync_oneshot sync_en (.clock(drp_clock), .in(drp_en_clock), .out(drp_en));

   always @ (posedge clock)
     begin
	dout[16] <= state != 0;
	case(state)
	  0: state <= write;
	  12: state <= write ? 1'b1 : state + drp_ready_sync;
	  default: state <= state + 1'b1;
	endcase
	drp_en_clock <= (state > 1) && (state < 12);
	if(write)
	  begin
	     drp_di <= din[15:0];
	     drp_address <= din[24:16];
	     drp_we <= din[31];
	  end
	if(state == 13)
	  dout[15:0] <= drp_do;
     end
endmodule
