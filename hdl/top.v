`timescale 1ns / 1ps

module vna_dsp
  (
   output [3:0]     pci_exp_txp,
   output [3:0]     pci_exp_txn,
   input [3:0] 	    pci_exp_rxp,
   input [3:0] 	    pci_exp_rxn,
   input 	    sys_clk_p,
   input 	    sys_clk_n,
   input 	    sys_rst_n,
   output reg [3:0] led = 4'h5
   );

   wire 	    clock;

   wire 	    pio_write_valid;
   wire [63:0] 	    pio_write_data;
   wire [12:0] 	    pio_address;
      
   reg [63:0] 	    fifo_to_pc_data = 0;
   reg 		    fifo_to_pc_write = 0;
   wire 	    fifo_to_pc_almost_full;

   wire [63:0] 	    fifo_from_pc_data;
   wire 	    fifo_from_pc_read = 1'b1;
   wire 	    fifo_from_pc_empty;
         
   pcie pcie
     (.pci_exp_txp(pci_exp_txp),
      .pci_exp_txn(pci_exp_txn),
      .pci_exp_rxp(pci_exp_rxp),
      .pci_exp_rxn(pci_exp_rxn),
      .sys_clk_p(sys_clk_p),
      .sys_clk_n(sys_clk_n),
      .sys_rst_n(sys_rst_n),
      .pio_clock(clock),
      .pio_write_valid(pio_write_valid),
      .pio_read_valid(),
      .pio_write_data(pio_write_data),
      .pio_address(pio_address),
      .pio_read_data(64'h0),
      .pio_read_data_valid(1'b0),      
      .fifo_clock(clock),
      .fifo_reset(),
      .fifo_to_pc_data(fifo_to_pc_data),
      .fifo_to_pc_write(fifo_to_pc_write),
      .fifo_to_pc_almost_full(fifo_to_pc_almost_full),
      .fifo_from_pc_data(fifo_from_pc_data),
      .fifo_from_pc_read(fifo_from_pc_read),
      .fifo_from_pc_empty(fifo_from_pc_empty)
      );
   
   always @ (posedge clock)
     begin
	if(pio_write_valid)
	  led[3:0] <= pio_write_data[3:0];

	if(~fifo_from_pc_empty)
	  begin
	     fifo_to_pc_write <= 1'b1;
	     fifo_to_pc_data <= fifo_from_pc_data;
	  end
	else
	  begin
	     fifo_to_pc_write <= 1'b0;
	  end
     end
      
endmodule

