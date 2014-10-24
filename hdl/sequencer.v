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
 *
 * clock must be same as FPC FIFO and TPC FIFO
 * reset is sync, active high
 * fpc_ signals go to a from PC fifo
 * tpc_ signals go to a to PC fifo
 *
 * User logic interface:
 * wvalid: indicates valid write data is available on wdata and address
 * rvalid: indicates a valid read request
 * address: address for read and write. qualified by wvalid or rvalid
 * wdata: write data. qualified by wvalid
 * rdata: read data, present read data here RPIPE cycles after rvalid.
 * status: inputs for wait condition instruction
 *
 * sequencer instructions:
 * NOP: fpc_data[63:0] = 0
 * WRITE: fpc_data[63:62] = 2
 *        fpc_data[ABITS-1:0] = address
 *        fpc_data[61] = increment
 *        fpc_data[47:32] = count
 *        fpc_data[31:0] = address
 * READ:
 *        fpc_data[63:62] = 3
 *        fpc_data[61] = increment
 *        fpc_data[47:32] = count
 *        fpc_data[31:0] = address
 * WAIT:
 *        fpc_data[63:62] = 1
 *        fpc_data[31:0] = timeout (clock cycles)
 *        fpc_data[39:32] = status bit
 *        fpc_data[41:40] = mode
 *           mode = 0: unconditional wait
 *           mode = 2: wait for status bit to be clear
 *           mode = 3: wait for status btt to be set
 */

module sequencer
  (
   input 		  clock,
   input 		  reset,
   // FPC FIFO
   output 		  fpc_read,
   input 		  fpc_valid,
   input [63:0] 	  fpc_data,
   // TPC FIFO
   input 		  tpc_ready,
   output 		  tpc_write,
   output [63:0] 	  tpc_data,
   // user logic interface
   output reg 		  rvalid = 0,
   output reg 		  wvalid = 0,
   output reg [ABITS-1:0] address = 0,
   output reg [DBITS-1:0] wdata = 0,
   input [DBITS-1:0] 	  rdata,
   input [SBITS-1:0] 	  status
   );

   parameter RPIPE = 2; // > 2
   parameter ABITS = 16; // 1 to 32
   parameter DBITS = 64; // 1 to 64
   parameter SBITS = 16; // 1 to 64
   parameter CBITS = 24; // 2 to 28

   reg [CBITS-1:0] 	  count = 32'hDEADBEEF;
   reg [1:0] 		  state = 0;
   reg 			  inc = 0;

   wire 		  rvalid_next = (state == 3) && tpc_ready;
   wire 		  wvalid_next = (state == 2) && fpc_read;

   assign fpc_read = fpc_valid && ((state == 0) || (state == 2));

   always @ (posedge clock)
     begin
	rvalid <= rvalid_next;
	wvalid <= wvalid_next;
	address <= (state == 0) ? fpc_data[ABITS-1:0] :
		   address + (inc && (rvalid || wvalid));
	wdata <= fpc_data[DBITS-1:0];
	inc <= (state == 0) ? fpc_data[61] : inc;
	case(state)
	  0: count <= fpc_data[CBITS+31:32];
	  1: count <= count - 1'b1;
	  2: count <= count - wvalid_next;
	  3: count <= count - rvalid_next;
	endcase
	if(reset)
	  state <= 2'd0;
	else
	  begin
	     case(state)
	       0: state <= fpc_read ? fpc_data[63:62] : 2'd0; // idle, nop
	       // wait, read, write
	       default: state <= (count[CBITS-1:1] == 0) ? 2'd0 : state;
	     endcase
	  end
     end
   // delay tpc_write RPIPE cycles from rvalid to allow for a read pipeline
   delay_n #(.N(RPIPE)) delay_tpc_write
     (.clock(clock), .in(rvalid), .out(tpc_write));
   assign tpc_data = rdata;

endmodule

module delay_n
  (
   input  clock,
   input  in,
   output out
   );

   parameter N = 2;

   reg [N-1:0] sreg = 0;

   assign out = sreg[N-1];

   always @ (posedge clock)
     sreg <= {sreg[N-2:0],in};

endmodule
