/* 
 * HIFIFO: Harmon Instruments PCI Express to FIFO
 * Copyright (C) 2014 Harmon Instruments, LLC
 * Author: Darrell Harmon
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

module sequencer
  (
   input 	 clock,
   input 	 reset,
   // FPC FIFO   
   output 	 fpc_read,
   input 	 fpc_valid,
   input [63:0]  fpc_data,
   // TPC FIFO
   input 	 tpc_ready,
   output 	 tpc_write,
   output [63:0] tpc_data   
   );

   sequencer_fpc sequencer_fpc
     (.clock(clock),
      .reset(reset),
      .fpc_read(fpc_read),
      .fpc_valid(fpc_valid),
      .fpc_data(fpc_data),
      .data(tpc_data),
      .c_valid(),
      .d_addr(),
      .d_valid(tpc_write),
      .d_ready({15'd0, tpc_ready}));
endmodule

module sequencer_fpc
  (
   input 	     clock,
   input 	     reset,
   output 	     fpc_read,
   input 	     fpc_valid,
   input [63:0]      fpc_data,
   output reg [63:0] data,
   output reg 	     c_valid,
   output reg [7:0]  d_addr,
   output reg 	     d_valid,
   input [15:0]      d_ready);

   reg 		     waiting_cycles, waiting_words;
   reg [31:0] 	     count = 32'hDEADBEEF;

   wire 	     c_valid_next = fpc_valid && ~waiting_words && ~waiting_cycles;   
   wire 	     start_wait_cycles = (c_valid_next && (fpc_data[63:60] == 1));
   wire 	     start_wait_words  = (c_valid_next && (fpc_data[63:60] == 2));

   wire 	     stall_words = ~d_ready[0];
   
   assign fpc_read = fpc_valid && ~waiting_cycles && ~(waiting_words && stall_words);
   
   always @ (posedge clock)
     begin
	d_valid <= fpc_valid && waiting_words && fpc_read;
	c_valid <= c_valid_next;
	waiting_cycles <= reset ? 1'b0 : (waiting_cycles && (count != 0)) || start_wait_cycles;
	waiting_words  <= reset ? 1'b0 : (waiting_words  && (count != 0)) || start_wait_words;
	count <= c_valid_next ? fpc_data[31:0] : count - ((count != 0) && ~(waiting_words && ~fpc_read));
	data <= fpc_data;
     end
   
endmodule
