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
   input 		  clock,
   input 		  reset,
   output reg [LMSB-BS:0] request_count,
   output [AMSB:0] 	  request_addr,
   output 		  request_valid,
   input 		  request_ack,
   input [DMSB:0] 	  wdata,
   input 		  wvalid,
   input 		  rc_last,
   output 		  rr_valid, // RR0 is descriptor fetch
   output [AMSB:0] 	  rr_addr,
   input 		  rr_ready,
   output [SMSB:0] 	  status,
   output [1:0] 	  interrupt
   );

   parameter BS = 0; // bit shift, number of LSBs to ignore in address
   parameter AMSB = 63; // address MSB
   parameter DMSB = 63; // data MSB
   parameter SMSB = 31; // status MSB
   parameter LMSB = 21; // transfer length MSB
            
   reg [1:0] 		  state;
   reg [AMSB-9:0] 	  next_desc_addr;
   reg [AMSB-BS:0] 	  addr_high;
   reg [SMSB-BS:0] 	  byte_count;
   reg [SMSB-BS:0] 	  interrupt_matchval;
   reg 			  reset_or_abort;
   reg 			  abort = 0;
   
   wire [DMSB:0] 	  desc_data;
   wire 		  desc_ready;
   wire 		  fifo_ready;

   wire 		  write_interrupt = wvalid && (wdata[2:0] == 1);
   wire 		  write_fifo      = wvalid && (wdata[2:1] == 1); // 2 or 3
   wire 		  write_desc_addr = wvalid && (wdata[3:0] == 4);
   wire 		  write_abort     = wvalid && (wdata[3:0] == 5);
   
   assign rr_addr = {next_desc_addr, 9'd0};
   assign rr_valid = (state == 2);
   assign request_addr = {addr_high,{BS{1'b0}}};
   assign request_valid = request_count != 0;
   assign status = {byte_count, {BS-2{1'b0}}, request_valid, (state == 0)};
   
   always @ (posedge clock)
     begin
	reset_or_abort <= reset | abort;
	byte_count <= reset ? 1'b0 : byte_count + request_ack;
	
	if(write_abort)
	  abort <= wdata[8];
	
	if(desc_ready && (desc_data[2:0] == 2) && (request_count == 0))
	  addr_high <= desc_data[AMSB:BS];
	else
	  addr_high <= addr_high + request_ack;
	
	if(write_interrupt)
	  interrupt_matchval <= wdata[SMSB:BS];
	
	if(reset_or_abort)
	  request_count <= 1'b0;
	else if(desc_ready && (desc_data[2:0] == 3) && (request_count == 0))
	  request_count <= desc_data[21:BS];
	else
	  request_count <= request_count - request_ack;
	
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
	  next_desc_addr <= wdata[AMSB:9];
     end

   one_shot one_shot_i0(.clock(clock), .in(interrupt_matchval == byte_count), .out(interrupt[0]));
   one_shot one_shot_i1(.clock(clock), .in(state == 0), .out(interrupt[1]));

   fwft_fifo #(.NBITS(AMSB+1)) fifo
     (
      .reset(reset_or_abort),
      .i_clock(clock),
      .i_data(wdata),
      .i_valid(write_fifo),
      .i_ready(fifo_ready),
      .o_clock(clock),
      .o_read(desc_ready & (request_count == 0)),
      .o_data(desc_data),
      .o_valid(desc_ready),
      .o_almost_empty()
      );
endmodule