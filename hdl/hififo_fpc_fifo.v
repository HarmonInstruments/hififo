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
   input 	    clock,
   input 	    reset,
   output [31:0]    status,
   output reg [1:0] interrupt,
   input [1:0] 	    fifo_number,
   // read completion
   input [63:0]     rx_data,
   input [7:0] 	    rc_tag,
   input [5:0] 	    rc_index,
   input 	    rc_valid,
   input 	    pio_wvalid,
   // read request
   output 	    rr0_valid, // RR0 is descriptor fetch
   output [63:0]    rr0_addr,
   input 	    rr0_ready,
   output reg 	    rr1_valid, // RR1 is data fetch
   output [63:0]    rr1_addr,
   input 	    rr1_ready,
   output [2:0]     rr1_tag,
   // FIFO
   input 	    fifo_clock, // for all FIFO signals
   input 	    fifo_read,
   output [63:0]    fifo_read_data,
   output 	    fifo_read_valid
   );

   // FIFO
   reg 		    enabled;
   reg [54:0] 	    addr;
   reg [12:0] 	    count;
   reg [1:0] 	    rr_holdoff = 0;
   reg [7:0] 	    block_filled = 0;
   reg [8:0] 	    p_read;
   reg [2:0] 	    p_write = 0; // 512 bytes
   reg [2:0] 	    p_request = 0; // 512 byes
   reg [22:0] 	    byte_count;
   reg 		    fifo_write_0, fifo_write_1;
   
   reg [22:0] 	    interrupt_matchval = 0;
   reg [1:0] 	    interrupt_match = 0;
   reg 		    interrupt_enable = 0;

   wire 	    desc_read = desc_ready & ~enabled;
   wire 	    desc_ready;
   wire [63:0] 	    desc_data;

   wire 	    desc_addr_ready;
   reg 		    desc_addr_ready_prev;
         
   wire [63:0] 	    data_fifo_in_data;
   wire 	    data_fifo_ready;
         
   // write enables
   wire 	    rx_valid = pio_wvalid || (rc_valid && rc_tag[7] && (rc_tag[2:0] == fifo_number));
   wire 	    write_interrupt = rx_valid && (rx_data[2:0] == 3);
   wire 	    write_reorder = (rc_tag[7:4] == fifo_number) && rc_valid;
   wire 	    write_last = rc_valid && (rc_tag[7:4] == fifo_number) && (rc_index == 6'h3F);
   wire 	    rc_last = (rc_valid && rc_tag[7] && (rc_tag[2:0] == fifo_number) && (rc_index[5:0] == 63));
   wire [2:0] 	    n_requested = p_request - p_read[8:6];
   wire 	    request_fifo_read = 0;

   assign rr1_tag = p_request[2:0];
   assign rr1_addr = {addr, 9'd0};
   assign status = {byte_count, 8'd0, desc_addr_ready};
               
   always @ (posedge clock)
     begin
	if(reset)
	  enabled <= 1'b0;
	else if(desc_ready && (desc_data[2:0] == 2))
	  enabled <= 1'b1;
	else if(count == 0)
	  enabled <= 1'b0;
	
	byte_count <= reset ? 1'b0 : byte_count + write_last;
	rr_holdoff <= reset ? 1'b0 : rr1_ready ? 2'd3 : rr_holdoff - (rr_holdoff != 0);
	p_read <= reset ? 1'b0 : p_read + ((p_read[8:6] != p_write[2:0]) && data_fifo_ready);
	p_write <= reset ? 1'b0 : p_write + (block_filled[p_write[2:0]]);
	p_request <= reset ? 1'b0 : p_request + (rr1_ready && rr1_valid);
	fifo_write_0 <= ((p_read[8:6] != p_write[2:0]) && data_fifo_ready);
	fifo_write_1 <= fifo_write_0;
	rr1_valid <= enabled && (n_requested < 6) && (rr_holdoff == 0);
	
	addr <= (desc_data[2:0] == 2) && ~enabled ? desc_data[63:9] : addr + rr1_ready;
	count <= (desc_data[2:0] == 1) && ~enabled ? desc_data[21:9] : count - rr1_ready;
	// interrupt
	if(reset | interrupt[0])
	  interrupt_enable <= 1'b0;
	else if(write_interrupt)
	  interrupt_enable <= 1'b1;
	if(write_interrupt)
	  interrupt_matchval <= rx_data[31:9];
	interrupt_match <= {interrupt_match[0], (interrupt_matchval == byte_count)};
	interrupt[0] <= interrupt_enable && (interrupt_match == 2'b01);
	desc_addr_ready_prev <= desc_addr_ready;
	interrupt[1] <= desc_addr_ready & ~desc_addr_ready_prev;
     end

   genvar 	 i;
   generate
      for (i = 0; i < 8; i = i+1) begin: block_fill
         always @(posedge clock) begin
	    if(reset)
	      block_filled[i] <= 1'b0;
	    else if(write_last && (rc_tag[3:0] == i))
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

   hififo_fetch_descriptor fetch_descriptor
     (
      .clock(clock),
      .reset(reset),
      .ready(desc_addr_ready),
      .write_fifo(rx_valid && ((rx_data[2:0] == 1) || (rx_data[2:0] == 2))),
      .write_desc_addr(rx_valid && (rx_data[2:0] == 4)),
      .wdata(rx_data),
      .rc_last(rc_last),
      .rr_valid(rr0_valid),
      .rr_addr(rr0_addr),
      .rr_ready(rr0_ready),
      .desc_ready(desc_ready),
      .desc_data(desc_data),
      .desc_read(desc_read)
      );
     
endmodule

module hififo_fetch_descriptor
  (
   input 	 clock,
   input 	 reset,
   output 	 ready,
   input 	 write_fifo,
   input 	 write_desc_addr,
   input [63:0]  wdata,
   input 	 rc_last,
   output 	 rr_valid, // RR0 is descriptor fetch
   output [63:0] rr_addr,
   input 	 rr_ready,
   output [63:0] desc_data,
   output 	 desc_ready,
   input 	 desc_read
   );
   
   reg [1:0] 	 state;
   reg [54:0] 	 next_desc_addr;
  
   wire 	 fifo_ready;
      
   assign rr_addr = {next_desc_addr, 9'd0};
   assign rr_valid = (state == 2);
   assign ready = (state == 0);
      
   always @ (posedge clock)
     begin
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
	  next_desc_addr <= wdata[63:9];
     end

   fwft_fifo #(.NBITS(64)) fifo
     (
      .reset(reset),
      .i_clock(clock),
      .i_data(wdata),
      .i_valid(write_fifo),
      .i_ready(fifo_ready),
      .o_clock(clock),
      .o_read(desc_read),
      .o_data(desc_data),
      .o_valid(desc_ready),
      .o_almost_empty()
      );
endmodule