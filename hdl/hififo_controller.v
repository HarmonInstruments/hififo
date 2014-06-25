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
   output reg [63:0] request_addr = 0,
   // to PC (tpc)
   output reg [0:0]  tpc_reset = 0,
   output reg 	     tx_wr_valid = 0,
   input 	     tx_wr_ready,
   input [0:0] 	     tpc_ready,
   output reg [0:0]  tpc_read = 0,
   output reg [8:0]  tpc_read_address = 0,
   // from PC (fpc)
   output reg [0:0]  fpc_reset = 0,
   output reg 	     tx_rr_valid = 0,
   output reg [7:0]  tx_rr_tag = 0,
   input 	     tx_rr_ready,
   input [0:0] 	     fpc_rr_valid,
   output reg [0:0]  fpc_rr_ready = 0,
   // status
   output reg 	     interrupt = 0
   );

   reg [42:0] 		    pagetable [511:0];
   reg [42:0] 		    pt_q;
            
   // PIO control
   always @ (posedge clock)
     begin
	casex({~pio_write_valid,pio_address})
	  13'h0000: interrupt <= pio_write_data[0];
	  13'h1xxx: pagetable[pio_address[8:0]] <= pio_write_data[63:21];
	endcase
	case({~pio_read_valid,pio_address})
	  13'h0000: pio_read_data <= 256;
	endcase
	pio_read_done <= pio_read_valid;
     end
   
endmodule

