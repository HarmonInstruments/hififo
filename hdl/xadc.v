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
 * din[22:16] DRP address
 * din[23] DRP write enale
 * 
 * dout[15:0] DRP data
 * dout[16] busy
 * 
 */

`timescale 1ns / 1ps

module xadc
  (
   input 	     clock,
   input 	     write,
   input [63:0]      din,
   output reg [16:0] dout = 0
   );

`ifndef SIM   
   reg [1:0] 	    state = 0;
   reg [2:0] 	    clkdiv = 0;
   
   reg [23:0] 	    drp_in;
   reg 		    drp_en = 0;
   wire [15:0] 	    drp_do;
   wire 	    drp_drdy;
            
   always @ (posedge clock)
     begin
	clkdiv <= clkdiv + 1'b1;
	dout[16] <= state != 0;
	state <= write ? 2'd1 :
		 (state == 1) && (clkdiv == 0) ? 2'd2 :
		 (state[1] == 1) && (clkdiv == 0) && drp_drdy ? 2'd0 : // 2 or 3
		 (state == 2) && (clkdiv == 0) ? 2'd3 :
		 state;
	drp_en <= (state == 1) && (clkdiv == 0) ? 1'b1 :
		  (state == 2) && (clkdiv == 0) ? 1'b0 :
		  drp_en;
	if(write)
	  drp_in <= din[23:0];
	if(state[1] && drp_drdy)
	  dout[15:0] <= drp_do;
     end

   XADC #(.INIT_42(16'h0400)) xadc_i
     (
      .ALM(), .OT(), .BUSY(), .CHANNEL(), .EOC(), .EOS(),
      .JTAGBUSY(), .JTAGLOCKED(), .JTAGMODIFIED(), .MUXADDR(),
      .VAUXN(), .VAUXP(), .VN(), .VP(), // analog
      .CONVST(1'b0), .CONVSTCLK(1'b0), .RESET(1'b0),
      // DRP outputs
      .DO(drp_do),
      .DRDY(drp_drdy),
      // DRP inputs
      .DADDR(drp_in[22:16]),
      .DCLK(clkdiv[1]),
      .DEN(drp_en),
      .DI(drp_in[15:0]),
      .DWE(drp_in[23])
      );
`endif
  
endmodule
