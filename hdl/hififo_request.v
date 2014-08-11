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
   input [15:0]  pci_id,
   output [7:0]  interrupt,
   output [7:0]  status,
   // PIO
   input 	 pio_wvalid,
   input [63:0]  pio_wdata,
   input [5:0] 	 pio_addr,
   // read request to pcie_tx
   output 	 rr_valid,
   output [65:0] rr_data,
   input 	 rr_ready,
   // read completion
   input 	 rc_valid,
   input [7:0] 	 rc_tag,
   input [5:0] 	 rc_index,
   input [63:0]  rc_data,
   // from request unit
   output [7:0]  r_valid,
   output [7:0]  r_abort,
   output [63:0] r_addr,
   output [18:0] r_count, // 8 bytes
   input [7:0] 	 r_ready
   );
   parameter ENABLE = 8'b00010001;
   
   reg [2:0] 	 state = 0;
   wire [5:0] 	 addr_in[0:7];
   wire [5:0] 	 addr_out[0:7];
   wire [63:0] 	 rr_addr[0:7];
   reg [3:0] 	 rr_state = 0;
   wire [7:0] 	 next_desc_request;
   
   genvar 	 i;
   generate
      for (i = 0; i < 8; i = i+1) begin: fifo
	 if((2**i & ENABLE) != 0)
	   begin: enabled
	      reg [54:0] next_desc_addr;
	      reg 	 next_desc_valid, next_desc_requested;
	      reg 	 abort;
	      reg [5:0]  p_in;
	      reg [5:0]  p_out;
	      reg [2:0]  read_delay = 0;
	      reg 	 reset_local;
	      reg 	 desc_ready, desc_ready_prev;
	      wire [5:0] count = p_in - p_out;
	      wire 	 read_a = (state == i) && r_ready[i] && (count != 0);
	      wire 	 write_addr = (pio_wvalid && (pio_addr == 16+2*i)) || (rc_valid && rc_tag[7] && (rc_tag[2:0] == i) && (rc_data[2:0] == 4));
	      wire 	 rc_last = (rc_valid && rc_tag[7] && (rc_tag[2:0] == i) && (rc_index[5:0] == 63));
	      assign rr_addr[i] = {next_desc_addr, 9'd0};
	      assign addr_in[i] = p_in;
	      assign addr_out[i] = p_out;
	      assign r_valid[i] = read_delay[2];
	      assign r_abort[i] = reset_local;
	      assign next_desc_request[i] = (count < 32) && next_desc_valid && ~next_desc_requested;
	      assign status[i] = desc_ready;
	      assign interrupt[i] = desc_ready & ~desc_ready_prev;
	      always @ (posedge clock)
		begin
		   if(write_addr)
		     next_desc_addr <= rc_data[63:9];
		   if(reset)
		     abort <= 1'b1;
		   else if(pio_wvalid && (pio_addr == 16+2*i))
		     abort <= pio_wdata[2:0] == 5;
		   desc_ready <= ~next_desc_valid & ~next_desc_requested;
		   desc_ready_prev <= desc_ready;
		   next_desc_valid <= reset ? 1'b0 : write_addr | (next_desc_valid & ~next_desc_requested);
		   next_desc_requested <= (reset | write_addr | rc_last) ? 1'b0 :
					  (rr_state == 2*i+1) ? 1'b1 : next_desc_requested;
		   reset_local <= reset | abort;
		   p_in <= reset_local ? 1'b0 : p_in + (rc_valid && rc_tag[7] && (rc_tag[2:0] == i) && (rc_data[2:0] == 2));
		   p_out <= reset_local ? 1'b0 : p_out + read_a;
		   read_delay <= {read_delay[1:0], read_a};
		end
	   end
	 else
	   begin
	      assign next_desc_request[i] = 0;
	      assign rr_addr[i] = 0;
	      assign addr_out[i] = 0;
	      assign addr_in[i] = 0;
	      assign r_valid[i] = 0;
	      assign r_abort[i] = 1;
	      assign status[i] = 0;
	      assign interrupt[i] = 0;
	   end
      end
   endgenerate
   
   wire [7:0]  rr_tag = {4'h8, 1'b0, rr_state[3:1]};
   wire [63:0] rr_addr_s = rr_addr[rr_state[3:1]];
   wire        rr_is_32 = rr_addr_s[63:32] == 0;
   wire [31:0] rr_dw0 = {2'd0, ~rr_is_32, 29'd128};
   wire [31:0] rr_dw1 = {pci_id, rr_tag[7:0], 8'hFF};
   wire [31:0] rr_dw2 = rr_is_32 ? rr_addr_s[31:0] : rr_addr_s[63:32];
   wire [31:0] rr_dw3 = rr_addr_s[31:0];
   reg [65:0]  rr_next = 0;
   reg [8:0]   ram_addr;
   
   assign rr_valid = rr_state[0];
   assign rr_data = rr_next;
      
   always @ (posedge clock)
     begin
	if(reset)
	  rr_state <= 1'b0;
	else if(rr_state[0])
	  rr_state <= rr_state + rr_ready;
	else
	  rr_state <= rr_state + (next_desc_request[rr_state[3:1]] ?  3'd1 : 3'd2);
	rr_next <= {rr_ready & rr_is_32, rr_ready, rr_ready ? {rr_dw3, rr_dw2} : {rr_dw1, rr_dw0}};
	state <= state + 1'b1;
	ram_addr <= {state, addr_out[state]};
     end
   
   block_ram #(.DBITS(19), .ABITS(9)) bram_count
     (.clock(clock),
      .w_data(rc_data[21:3]),
      .w_valid(rc_valid && rc_tag[7] && (rc_data[2:0] == 1)),
      .w_addr({rc_tag[2:0], addr_in[rc_tag[2:0]]}),
      .r_data(r_count),
      .r_addr(ram_addr)
      );

   block_ram #(.DBITS(61), .ABITS(9)) bram_addr
     (.clock(clock),
      .w_data(rc_data[63:3]),
      .w_valid(rc_valid && rc_tag[7] && (rc_data[2:0] == 2)),
      .w_addr({rc_tag[2:0], addr_in[rc_tag[2:0]]}),
      .r_data(r_addr[63:3]),
      .r_addr(ram_addr)
      );

   assign r_addr[2:0] = 0;
      
endmodule