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
   input 	    clock,
   input 	    reset,
   input [15:0]     pci_id,
   output [31:0]    status,
   output reg [1:0] interrupt,
   // writes and read completions
   input [63:0]     rx_data,
   input 	    rx_data_valid,
   input 	    rc_last,
   // to PCI TX
   output 	    rr_valid,
   output [63:0]    rr_addr,
   input 	    rr_ready,
   output reg 	    wr_valid = 0,
   input 	    wr_ready,
   output [63:0]    wr_data,
   output [63:0]    wr_addr,
   output reg 	    wr_last,
   output [4:0]     wr_count,
   // FIFO
   input 	    fifo_clock,
   input 	    fifo_write,
   input [63:0]     fifo_data,
   output 	    fifo_ready
   );

   reg [60:0] 	    addr; // 8 bytes
   reg [19:0] 	    count; // 8 bytes
   reg [4:0] 	    state = 0;
   reg [28:0] 	    byte_count;
   reg 		    desc_addr_ready_prev;
   reg 		    enabled;
      
   reg [28:0] 	    interrupt_matchval = 0;
   reg [1:0] 	    interrupt_match;
   reg 		    interrupt_0_enable = 0;

   wire 	    o_almost_empty;
   wire [63:0] 	    desc_data;
   wire 	    desc_ready, desc_addr_ready;
   wire             desc_read = desc_ready & ~enabled;
   wire 	    write_interrupt = rx_data_valid && (rx_data[2:0] == 4);
   wire 	    write_abort = rx_data_valid && (rx_data[2:0] == 5);
   
   wire 	    fifo_read = (wr_ready && wr_valid) || ((state != 0) && (state < 30));
      
   assign wr_addr = {addr, 3'd0};
   assign status = {byte_count, 2'd0, desc_addr_ready};
   assign wr_count = 16;
   
               
   always @ (posedge clock)
     begin
	byte_count <= reset ? 1'b0 : byte_count + fifo_read;
	wr_valid <= reset ? 1'b0 : (((state == 0) || (state > 30)) && enabled && ~o_almost_empty && (count != 0));
	if(reset)
	  state <= 1'b0;
	else if(state == 0)
	  state <= wr_ready ? 5'd15 : 5'd0;
	else
	  state <= state + 1'b1;

	if(reset)
	  enabled <= 1'b0;
	else if(desc_ready && (desc_data[2:0] == 2))
	  enabled <= 1'b1;
	else if(count == 0)
	  enabled <= 1'b0;
	wr_last <= state == 29;
	
	addr <= (desc_data[2:0] == 2) && ~enabled ? desc_data[63:3] : addr + fifo_read;
	count <= (desc_data[2:0] == 1) && ~enabled ? desc_data[21:3] : count - fifo_read;

	// interrupt
	if(reset | interrupt[0])
	  interrupt_0_enable <= 1'b0;
	else if(write_interrupt)
	  interrupt_0_enable <= 1'b1;
	if(write_interrupt)
	  interrupt_matchval <= rx_data[31:3];
	interrupt_match <= {interrupt_match[0], (interrupt_matchval == byte_count)};
	desc_addr_ready_prev <= desc_addr_ready;
	interrupt[0] <= interrupt_0_enable && (interrupt_match == 2'b01);
	interrupt[1] <= desc_addr_ready & ~desc_addr_ready_prev;
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

    hififo_fetch_descriptor fetch_descriptor
     (
      .clock(clock),
      .reset(reset),
      .ready(desc_addr_ready),
      .write_fifo(rx_data_valid && ((rx_data[2:0] == 1) || (rx_data[2:0] == 2))),
      .write_desc_addr(rx_data_valid && (rx_data[2:0] == 4)),
      .wdata(rx_data),
      .rc_last(rc_last),
      .rr_valid(rr_valid),
      .rr_addr(rr_addr),
      .rr_ready(rr_ready),
      .desc_ready(desc_ready),
      .desc_data(desc_data),
      .desc_read(desc_read)
      );

endmodule
