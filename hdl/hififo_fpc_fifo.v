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

module pcie_from_pc_fifo
  (
   input 	    clock,
   input 	    reset,
   output reg [1:0] interrupt = 0,
   output [31:0]    status,
   // PIO
   input 	    pio_wvalid,
   input [63:0]     pio_wdata,
   input [3:0] 	    pio_addr, 
   // read completion
   input 	    rc_valid,
   input [7:0] 	    rc_tag,
   input [5:0] 	    rc_index,
   input [63:0]     rc_data,
   // read request
   output 	    rr_valid,
   output [63:0]    rr_addr,
   input 	    rr_ready,
   // FIFO
   input 	    fifo_clock, // for all FIFO signals
   input 	    fifo_read,
   output [63:0]    fifo_read_data,
   output 	    fifo_read_valid
   );

   reg [7:0] 	    block_filled = 0;
   reg [8:0] 	    p_read;
   reg [16:0] 	    p_write = 0; // 512 bytes
   reg [16:0] 	    p_request = 0; // 512 byes
   reg [16:0] 	    p_stop = 0; // 512 bytes
   reg [16:0] 	    p_int = 0; // 512 bytes
   wire 	    write = (rc_tag[7:3] == 0) && rc_valid;
   wire [8:0] 	    write_address = {rc_tag[2:0],rc_index};
   wire 	    write_last = write && (rc_index == 6'h3F);
   wire [2:0] 	    prp2 = p_request[2:0] + 2'd2;
   assign rr_valid = ~rr_ready & (prp2 != p_read[8:6]) & (p_request != p_stop);
   assign rr_addr = {p_request[16:0], 9'd0};
   assign status = {p_write, 9'd0};
   wire [63:0] 	    fifo_write_data;
   wire 	    fifo_ready;
   reg 		    fifo_write_0, fifo_write_1;

   genvar 	    i;
   generate
      for (i = 0; i < 8; i = i+1) begin: block_fill
         always @(posedge clock)
	   block_filled[i] <= reset ? 1'b0 : (write_last && (rc_tag == i)) || ((p_write[2:0] == i) ? 1'b0 : block_filled[i]);
      end
   endgenerate   
   
   always @ (posedge clock)
     begin
	p_stop <= reset ? 1'b0 : (pio_wvalid && (pio_addr == 6) ? pio_wdata[25:9] : p_stop);
	p_int <= pio_wvalid && (pio_addr == 7) ? pio_wdata[25:9] : p_int;
	p_read <= reset ? 1'b0 : p_read + ((p_read[8:6] != p_write[2:0]) && fifo_ready);
	p_write <= reset ? 1'b0 : p_write + block_filled[p_write[2:0]];
	p_request <= reset ? 1'b0 : p_request + rr_ready;
	interrupt <= {(p_stop == p_write), (p_int == p_write)};
	fifo_write_0 <= ((p_read[8:6] != p_write[2:0]) && fifo_ready);
	fifo_write_1 <= fifo_write_0;
     end

   block_ram #(.DBITS(64), .ABITS(9)) bram_reorder
     (.clock(clock),
      .w_data(rc_data),
      .w_valid(write),
      .w_addr(write_address),
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
