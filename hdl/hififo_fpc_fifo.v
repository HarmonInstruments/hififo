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

module pcie_from_pc_fifo
  (
   input 	 clock,
   input 	 reset,
   output [31:0] status,
   input [1:0] 	 fifo_number,
   // read completion
   input 	 rc_valid,
   input [7:0] 	 rc_tag,
   input [5:0] 	 rc_index,
   input [63:0]  rc_data,
   // read request
   output 	 rr_valid,
   output [2:0]  rr_tag_low,
   input 	 rr_ready,
   // FIFO
   input 	 fifo_clock, // for all FIFO signals
   input 	 fifo_read,
   output [63:0] fifo_read_data,
   output 	 fifo_read_valid
   );

   reg [1:0] 	 rr_holdoff = 0;
   reg [7:0] 	 block_filled = 0;
   reg [8:0] 	 p_read;
   reg [2:0] 	 p_write = 0; // 512 bytes
   reg [2:0] 	 p_request = 0; // 512 byes
   reg [22:0] 	 byte_count;
   wire 	 write = (rc_tag[7:4] == fifo_number) && rc_valid;
   wire 	 write_last = write && (rc_index == 6'h3F);
   wire [2:0] 	 n_requested = p_request - p_write;
   assign rr_valid = (rr_holdoff == 0) && (rr_holdoff == 0) && (n_requested < 6);
   assign rr_tag_low = p_request[2:0];
   assign status = {byte_count, 9'd0};
   wire [63:0] 	 fifo_write_data;
   wire 	 fifo_ready;
   reg 		 fifo_write_0, fifo_write_1;

   genvar 	 i;
   generate
      for (i = 0; i < 8; i = i+1) begin: block_fill
         always @(posedge clock)
	   block_filled[i] <= reset ? 1'b0 : (write_last && (rc_tag[6:0] == i)) || ((p_write[2:0] == i) ? 1'b0 : block_filled[i]);
      end
   endgenerate   
   
   always @ (posedge clock)
     begin
	byte_count <= reset ? 1'b0 : byte_count + ((rc_valid) && (rc_index == 6'h3F));
	rr_holdoff <= reset ? 1'b0 : rr_ready ? 2'd3 : rr_holdoff - (rr_holdoff != 0);
	p_read <= reset ? 1'b0 : p_read + ((p_read[8:6] != p_write[2:0]) && fifo_ready);
	p_write <= reset ? 1'b0 : p_write + block_filled[p_write[2:0]];
	p_request <= reset ? 1'b0 : p_request + rr_ready;
	fifo_write_0 <= ((p_read[8:6] != p_write[2:0]) && fifo_ready);
	fifo_write_1 <= fifo_write_0;
     end

   block_ram #(.DBITS(64), .ABITS(9)) bram_reorder
     (.clock(clock),
      .w_data(rc_data),
      .w_valid(write),
      .w_addr({rc_tag[2:0],rc_index}),
      .r_data(fifo_write_data),
      .r_addr(p_read)
      );
      
   fwft_fifo #(.NBITS(64)) fpc_fifo
     (
      .reset(reset),
      .i_clock(clock),
      .i_data(fifo_write_data),
      .i_valid(fifo_write_1),
      .i_ready(fifo_ready),
      .o_clock(fifo_clock),
      .o_read(fifo_read),
      .o_data(fifo_read_data),
      .o_valid(fifo_read_valid),
      .o_almost_empty()
      );
   
endmodule
