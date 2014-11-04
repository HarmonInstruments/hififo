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
   output 	 interrupt,
   input [1:0] 	 fifo_number,
   // read completion
   input [63:0]  rx_data,
   input [7:0] 	 rc_tag,
   input [5:0] 	 rc_index,
   input 	 rc_valid,
   input 	 pio_wvalid,
   // read request
   output reg 	 rr_valid, // RR is data fetch
   output [63:0] rr_addr,
   input 	 rr_ready,
   output [2:0]  rr_tag,
   // FIFO
   input 	 fifo_clock, // for all FIFO signals
   input 	 fifo_read,
   output [63:0] fifo_read_data,
   output 	 fifo_read_valid
   );

   // FIFO
   reg [1:0] 	    rr_holdoff = 0;
   reg [7:0] 	    block_filled = 0;
   reg [8:0] 	    p_read;
   reg [2:0] 	    p_write = 0; // 512 bytes
   reg [2:0] 	    p_request = 0; // 512 byes
   reg 		    fifo_write_0, fifo_write_1;

   wire [63:0] 	    data_fifo_in_data;
   wire 	    data_fifo_ready;
   wire 	    request_valid;

   // write enables
   wire 	    rx_valid = pio_wvalid;
   wire 	    write_reorder = (rc_tag[5:3] == fifo_number) && rc_valid;
   wire 	    write_last = rc_valid
		    && (rc_tag[5:3] == fifo_number) && (rc_index == 6'h3F);
   wire [2:0] 	    n_requested = p_request - p_read[8:6];
   wire 	    request_fifo_read = 0;

   assign rr_tag = p_request[2:0];

   always @ (posedge clock)
     begin
	rr_holdoff <= reset ? 1'b0 :
		      rr_ready ? 2'd3 :
		      rr_holdoff - (rr_holdoff != 0);
	p_read <= reset ? 1'b0 :
		  p_read + ((p_read[8:6] != p_write[2:0]) && data_fifo_ready);
	p_write <= reset ? 1'b0 :
		   p_write + (block_filled[p_write[2:0]]);
	p_request <= reset ? 1'b0 :
		     p_request + (rr_ready && rr_valid);
	fifo_write_0 <= ((p_read[8:6] != p_write[2:0]) && data_fifo_ready);
	fifo_write_1 <= fifo_write_0;
	rr_valid <= request_valid && (n_requested < 6) && (rr_holdoff == 0);
     end

   genvar 	 i;
   generate
      for (i = 0; i < 8; i = i+1) begin: block_fill
         always @(posedge clock) begin
	    if(reset)
	      block_filled[i] <= 1'b0;
	    else if(write_last && (rc_tag[2:0] == i))
	      block_filled[i] <= 1'b1;
	    else if(p_write[2:0] == i)
	      block_filled[i] <= 1'b0;
	 end
      end
   endgenerate

   block_ram #(.DBITS(64), .ABITS(9)) bram_reorder
     (.clock(clock),
      .w_data(rx_data),
      .w_valid(write_reorder),
      .w_addr({rc_tag[2:0],rc_index}),
      .r_data(data_fifo_in_data),
      .r_addr(p_read)
      );

   fwft_fifo #(.NBITS(64)) fpc_fifo
     (
      .reset(reset),
      .i_clock(clock),
      .i_data(data_fifo_in_data),
      .i_valid(fifo_write_1),
      .i_ready(data_fifo_ready),
      .o_clock(fifo_clock),
      .o_read(fifo_read),
      .o_data(fifo_read_data),
      .o_valid(fifo_read_valid),
      .o_almost_empty()
      );

   hififo_fetch_descriptor #(.BS(9)) fetch_descriptor
     (
      .clock(clock),
      .reset(reset),
      .request_addr(rr_addr),
      .request_ack(rr_ready),
      .request_valid(request_valid),
      .wvalid(rx_valid),
      .wdata(rx_data),
      .status(status),
      .interrupt(interrupt)
      );

endmodule
