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
   input 	 clock,
   input 	 reset,
   output [31:0] status,
   output [1:0]  interrupt,
   // writes and read completions
   input [63:0]  rx_data,
   input 	 rx_data_valid,
   input 	 rc_last,
   // to PCI TX
   output 	 rr_valid,
   output [63:0] rr_addr,
   input 	 rr_ready,
   output reg 	 wr_valid = 0,
   input 	 wr_ready,
   output [63:0] wr_data,
   output [63:0] wr_addr,
   output reg 	 wr_last,
   output [4:0]  wr_count,
   // FIFO
   input 	 fifo_clock,
   input 	 fifo_write,
   input [63:0]  fifo_data,
   output 	 fifo_ready
   );

   reg [4:0] 	 state = 0;

   wire 	 o_almost_empty;
   wire 	 request_valid;
   wire 	 fifo_read = (wr_ready && wr_valid) || ((state != 0) && (state < 30));

   assign wr_count = 16;

   always @ (posedge clock)
     begin
	if(reset)
	  wr_valid <= 1'b0;
	else
	  wr_valid <= ((state == 0) || (state > 29)) && request_valid && ~o_almost_empty;
	wr_last <= state == 29;
	if(reset)
	  state <= 1'b0;
	else if(state == 0)
	  state <= wr_ready ? 5'd15 : 5'd0;
	else
	  state <= state + 1'b1;
     end

   fwft_fifo #(.NBITS(64)) data_fifo
     (
      .reset(reset),
      .i_clock(fifo_clock),
      .i_data(fifo_data),
      .i_valid(fifo_write),
      .i_ready(fifo_ready),
      .o_clock(clock),
      .o_read(fifo_read),
      .o_data(wr_data),
      .o_valid(),
      .o_almost_empty(o_almost_empty)
      );

    hififo_fetch_descriptor #(.BS(3)) fetch_descriptor
     (
      .clock(clock),
      .reset(reset),
      .request_count(),
      .request_addr(wr_addr),
      .request_valid(request_valid),
      .request_ack(fifo_read),
      .wvalid(rx_data_valid),
      .wdata(rx_data),
      .rc_last(rc_last),
      .rr_valid(rr_valid),
      .rr_addr(rr_addr),
      .rr_ready(rr_ready),
      .status(status),
      .interrupt(interrupt)
      );

endmodule
