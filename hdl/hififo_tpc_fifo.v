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

module hififo_tpc_fifo
  (
   input 	     clock,
   input 	     reset,
   input [15:0]      pci_id,
   output [31:0]     status,
   // from request unit
   input 	     r_valid,
   input 	     r_abort,
   input [60:0]      r_addr, // 8 bytes
   input [18:0]      r_count, // 8 bytes
   output 	     r_ready,
   // to PCI TX
   output reg 	     wr_valid = 0,
   input 	     wr_ready,
   output reg [65:0] wr_data,
   // FIFO
   input 	     fifo_clock,
   input 	     fifo_write,
   input [63:0]      fifo_data,
   output 	     fifo_ready
   );

   function [31:0] es; // endian swap
      input [31:0]   x;
      es = {x[7:0], x[15:8], x[23:16], x[31:24]};
   endfunction

   // clock
   reg [60:0] 	     addr; // 8 bytes
   reg [19:0] 	     count; // 8 bytes
   wire 	     o_almost_empty;
   wire 	     o_read = (wr_ready && wr_valid) || ((state != 0) && (state < 30));
   reg [63:0] 	     wr_data_next;
   wire 	     wr_valid_set = ~o_almost_empty && (count != 0);
   reg 		     is_32;
   wire [63:0] 	     wr_addr = {addr, 3'd0};
   wire [63:0] 	     o_data;
   reg [4:0] 	     state = 0;

   reg [28:0] 	     byte_count;
   
   assign status = {byte_count, 3'd0};
   assign r_ready = count == 0;
   
   always @ (posedge clock)
     begin
	byte_count <= reset ? 1'b0 : byte_count + o_read;
	if(reset)
	  count <= 1'b0;
	else if(r_valid)
	  count <= r_count;
	else if(o_read)
	  count <= count - 1'b1;
	if(r_valid)
	  addr <= r_addr;
	else if(o_read)
	  addr <= addr + 1'b1;
	wr_valid <= reset ? 1'b0 : (((state == 0) || (state > 30)) && wr_valid_set);
	if(reset)
	  state <= 1'b0;
	else if(state == 0)
	  state <= wr_ready ? 5'd15 : 5'd0;
	else
	  state <= state + 1'b1;
	if(wr_ready | (state != 0))
	  begin
	     wr_data[65:64] <= (state == 30) ? {is_32, 1'b1} : 2'b00;
	     wr_data[63:0] <= is_32 ? {es(o_data[31:0]), wr_data_next[31:0]} : wr_data_next;
	     wr_data_next <= is_32 ? es(o_data[63:32]) : {es(o_data[63:32]), es(o_data[31:0])};
	  end
	else
	  begin
	     is_32 <= wr_addr[63:32] == 0;
	     wr_data <= {2'b00, pci_id, 16'h00FF, 2'b01, wr_addr[63:32] != 0, 29'd32};
	     wr_data_next <= wr_addr[63:32] == 0 ? {32'h0, wr_addr[31:0]} : {wr_addr[31:0], wr_addr[63:32]};
	  end
     end

   fwft_fifo #(.NBITS(64)) tpc_fifo
     (
      .reset(reset),
      .i_clock(fifo_clock),
      .i_data(fifo_data),
      .i_valid(fifo_write),
      .i_ready(fifo_ready),
      .o_clock(clock),
      .o_read(o_read),
      .o_data(o_data),
      .o_valid(),
      .o_almost_empty(o_almost_empty)
      );
     
endmodule