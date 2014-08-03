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

module hififo_controller
  (
   input 	     clock,
   input 	     pci_reset,
   // PIO
   input 	     pio_write_valid,
   input 	     pio_read_valid,
   input [63:0]      pio_write_data,
   input [12:0]      pio_address,
   output reg [63:0] pio_read_data = 0,
   output reg 	     pio_read_done = 0,
   // common
   // to PC (tpc)
   output reg 	     tpc_reset = 0,
   output reg 	     tx_wr_valid = 0,
   input 	     tx_wr_ready, 
   output [63:0]     tx_wr_addr,
   // handshake with the FIFO
   input 	     tpc_ready,
   output reg 	     tpc_valid,
   // from PC (fpc)
   output reg [0:0]  fpc_reset = 0,
   output reg 	     tx_rr_valid = 0,
   output reg [63:0] tx_rr_addr,
   output reg [7:0]  tx_rr_tag = 0,
   input 	     tx_rr_ready,
   // handshake with the FIFO
   input 	     fpc_rr_valid,
   output reg 	     fpc_rr_ready = 0,
   // status
   output reg 	     interrupt = 0
   );

   reg [42:0] 	     tpc_pt [31:0];
   reg [42:0] 	     tpc_pt_q;
   reg [18:0] 	     tpc_pin;
   reg [18:0] 	     tpc_pout;
   reg [18:0] 	     tpc_stop;
   reg [18:0] 	     tpc_match;

   assign tx_wr_addr = {tx_pt_q, tpc_pout[13:0], 7'd0};
      
   // PIO control
   always @ (posedge clock)
     begin
	if(pio_write_valid && (pio_address == 0))
	  interrupt <= pio_write_data[0];
	if(pio_write_valid && (pio_address == 1))
	  tpc_stop <= pio_write_data[32:9];
	if(pio_write_valid && (pio_address == 2))
	  tpc_match <= pio_write_data[32:9];
	if(pio_write_valid && (pio_address[12:5] == 1))
	  tpc_pt[pio_address[4:0]] <= pio_write_data[63:21];
	case({~pio_read_valid,pio_address})
	  13'h0000: pio_read_data <= 256;
	endcase
	pio_read_done <= pio_read_valid;
	
	tpc_reset <= pci_reset;
	if(tpc_reset)
	  begin
	     tpc_stop <= 1'b0;
	     tpc_match <= 1'b0;
	     tpc_pin <= 1'b0;
	     tpc_pout <= 1'b0;
	  end
	else
	  begin
	     if((tpc_pin != tpc_pout) && tpc_valid && ~tx_wr_valid)
	       begin
		  tx_wr_valid <= 1'b1;
		  tpc_pt_q <= tpc_pt[tpc_pout[18:14]];
	       end
	     else if(tx_wr_ready)
	       begin
		  tx_wr_valid <= 1'b0;
	       end
	     tpc_ready <= tx_wr_ready;
	  end
     end
   
endmodule

