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

module hififo_request
  (
   input 	 clock,
   input 	 reset,
   // PIO for page table writes
   input 	 pio_wvalid,
   input [63:0]  pio_wdata,
   input [10:0]  pio_addr,
   // from request unit
   output [7:0]  r_valid,
   output [60:0] r_addr, // 8 bytes
   output [18:0] r_count, // 8 bytes
   input [7:0] 	 r_ready
   );
   parameter enables = 8'b00010001;
   
   reg [81:0] 	 req_ram[0:511];
   reg [8:0] 	 ram_addr;
   
   reg [2:0] 	 state = 0;
   wire [8:0] 	 addr_in[0:7];
   wire [8:0] 	 addr_out[0:7];
   
   genvar 	 i;
   generate
      for (i = 0; i < 8; i = i+1) begin: block_fill
	 if((2**i & enables) != 0)
	   begin
	      reg [5:0] p_in;
	      reg [5:0] p_out;
	      wire [5:0] count = p_in - p_out;
	      wire read_a = (state == i) && r_ready[i] && (count != 0);
	      reg  read_b = 0;
	      reg  read_c = 0;
	      reg  read_d = 0;
	      assign addr_in[i] = 64*i | p_in;
	      assign addr_out[i] = 64*i | p_out;
	      assign r_valid[i] = read_d;
	      always @ (posedge clock)
		begin
		   p_in <= reset ? 1'b0 : p_in + (pio_wvalid && (pio_addr[10:4] == 1) && (pio_addr[3:0] == 2*i + 1));
		   p_out <= reset ? 1'b0 : p_out + read_a;
		   read_b <= read_a;
		   read_c <= read_b;
		   read_d <= read_c;
		end
	   end
	 else
	   begin
	      assign addr_out[i] = 1'b0;
	      assign addr_in[i] = 1'b0;
	      assign r_valid[i] = 1'b0;
	   end
      end
   endgenerate
   
   reg [18:0] pio_high;
   
   always @ (posedge clock)
     begin
	state <= state + 1'b1;
        if(pio_wvalid & (pio_addr[10:4] == 1) & ~pio_addr[0])
	  pio_high <= pio_wdata[21:3];
	ram_addr <= addr_out[state];
     end
   
   block_ram #(.DBITS(80), .ABITS(9)) bram_req
     (.clock(clock),
      .w_data({pio_high, pio_wdata[63:3]}),
      .w_valid(pio_wvalid && (pio_addr[10:4] == 1) && pio_addr[0]),
      .w_addr(addr_in[pio_addr[3:1]]),
      .r_data({r_count,r_addr}),
      .r_addr(ram_addr)
      );
   
endmodule