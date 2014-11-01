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

module sync
  (
   input  clock,
   input  in,
   output out
   );

   (* ASYNC_REG="TRUE", TIG="TRUE" *) reg [2:0] sreg = 0;
   always @(posedge clock)
     sreg <= {sreg[1:0], in};
   assign out = sreg[2];
endmodule

module one_shot
  (
   input      clock,
   input      in,
   output reg out = 0
   );
   reg 	      in_prev = 0;
   always @(posedge clock)
     begin
	in_prev <= in;
	out <= in & ~in_prev;
     end
endmodule

module pulse_stretch
  (
   input      clock,
   input      in,
   output reg out = 0
   );
   parameter NB=3;
   reg [NB-1:0] count = 0;
   always @(posedge clock)
     begin
	if(in)
	  count <= 1'b1;
	else
	  count <= count + (count != 0);
	out <= in | (count != 0);
     end
endmodule

module sync_oneshot(input clock, input in, output out);
   wire synced;
   sync sync(.clock(clock), .in(in), .out(synced));
   one_shot one_shot(.clock(clock), .in(synced), .out(out));
endmodule

module sync_pulse (input clock_in, input clock_out, input in, output out);
   parameter NB = 3;
   wire stretched;
   pulse_stretch #(.NB(NB)) stretch
     (.clock(clock_in), .in(in), .out(stretched));
   sync_oneshot(.clock(clock_out), .in(stretched), .out(out));
endmodule