`timescale 1ns / 1ps

module vna_dsp
  (
   output [3:0] pci_exp_txp,
   output [3:0] pci_exp_txn,
   input [3:0] 	pci_exp_rxp,
   input [3:0] 	pci_exp_rxn,
   input 	sys_clk_p,
   input 	sys_clk_n,
   input 	sys_rst_n,
   output [3:0] led
   );

   pcie pcie
     (.pci_exp_txp(pci_exp_txp),
      .pci_exp_txn(pci_exp_txn),
      .pci_exp_rxp(pci_exp_rxp),
      .pci_exp_rxn(pci_exp_rxn),
      .sys_clk_p(sys_clk_p),
      .sys_clk_n(sys_clk_n),
      .sys_rst_n(sys_rst_n),
      .led(led)
      );

endmodule

