/* 
 * PCI Express to FIFO example
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

module vna_dsp
  (
   output [3:0]     pcie_txp,
   output [3:0]     pcie_txn,
   input [3:0] 	    pcie_rxp,
   input [3:0] 	    pcie_rxn,
   input 	    pcie_refclk_p,
   input 	    pcie_refclk_n,
   input 	    pcie_rst_n,
   // SPI configuration flash
   //output 	    cflash_sck,
   output 	    cflash_sdi,
   output 	    cflash_cs,
   input 	    cflash_sdo,
   output [1:0]     cflash_high,
   output reg [3:0] led = 4'h5
   );
   
   wire 	    clock;
   
   reg [63:0] 	    tpc_data = 0;
   reg 		    tpc_write = 0;
    
   wire [63:0] 	    fpc_data;
   reg 		    fpc_read = 1'b0;
   
   wire [7:0] 	    fifo_ready, fifo_reset;
   wire [63:0] 	    seq_fpc_data, seq_tpc_data;
   wire 	    seq_read, seq_write;

   reg [63:0] 	    count = 0;
   reg 		    count_write = 0;
   reg 		    null_read = 0;

   assign cflash_high = 2'b11;
   wire 	    cflash_sck;
   
   wire 	    seq_rvalid, seq_wvalid;
   wire [15:0] 	    seq_address;
   wire [63:0] 	    seq_wdata;
   reg [63:0] 	    seq_rdata0, seq_rdata1;
   reg [63:0] 	    seq_test;
   wire [7:0] 	    seq_spidata;
      
   hififo_pcie #(.ENABLE(8'b01110111)) hififo
     (.pci_exp_txp(pcie_txp),
      .pci_exp_txn(pcie_txn),
      .pci_exp_rxp(pcie_rxp),
      .pci_exp_rxn(pcie_rxn),
      .sys_clk_p(pcie_refclk_p),
      .sys_clk_n(pcie_refclk_n),
      .sys_rst_n(pcie_rst_n),
      .clock(clock),
      .fifo_clock({8{clock}}),
      .fifo_reset(fifo_reset),
      .fifo_ready(fifo_ready),
      .fifo_rw({1'b1, count_write, seq_write, tpc_write, fifo_ready[3], null_read, seq_read, fpc_read}),
      .fifo_data_0(fpc_data),
      .fifo_data_1(seq_fpc_data),
      .fifo_data_2(),
      .fifo_data_3(),
      .fifo_data_4(tpc_data),
      .fifo_data_5(seq_tpc_data),
      .fifo_data_6(count),
      .fifo_data_7(64'h0)  
      );

   sequencer #(.ABITS(16)) sequencer
     (.clock(clock),
      .reset(fifo_reset[1]),
      .fpc_read(seq_read),
      .fpc_valid(fifo_ready[1]),
      .fpc_data(seq_fpc_data),
      .tpc_ready(fifo_ready[5]),
      .tpc_write(seq_write),
      .tpc_data(seq_tpc_data),
      .rvalid(seq_rvalid),
      .wvalid(seq_wvalid),
      .address(seq_address),
      .wdata(seq_wdata),
      .rdata(seq_rdata1),
      .status(16'h0)
      );

   spi_8bit_rw spi_cflash
     (
      .clock(clock),
      .write(seq_wvalid && (seq_address == 4)),
      .din(seq_wdata[8:0]),
      .dout(seq_spidata),
      .cs(cflash_cs),
      .sck(cflash_sck),
      .mosi(cflash_sdi),
      .miso(cflash_sdo));

   `ifndef SIM
   (*keep="TRUE"*) STARTUPE2 STARTUPE2 
	  (
	   .CFGCLK(),             // 1-bit output: Configuration main clock output
	   .CFGMCLK(),            // 1-bit output: Configuration internal oscillator clock output
	   .EOS(),                // 1-bit output: Active high output signal indicating the End Of Startup.
	   .PREQ(),               // 1-bit output: PROGRAM request to fabric output
	   .CLK(1'b0),            // 1-bit input: User start-up clock input
	   .GSR(1'b0),            // 1-bit input: Global Set/Reset input (GSR cannot be used for the port name)
	   .GTS(1'b0),            // 1-bit input: Global 3-state input (GTS cannot be used for the port name)
	   .KEYCLEARB(1'b0),      // 1-bit input: Clear AES Decrypter Key input from Battery-Backed RAM (BBRAM)
	   .PACK(1'b0),           // 1-bit input: PROGRAM acknowledge input
	   .USRCCLKO(cflash_sck), // 1-bit input: User CCLK input
	   .USRCCLKTS(1'b0),      // 1-bit input: User CCLK 3-state enable input
	   .USRDONEO(1'b0),       // 1-bit input: User DONE pin output control
	   .USRDONETS(1'b0)       // 1-bit input: User DONE 3-state enable output
	   );
   `endif
   
   always @ (posedge clock)
     begin
	if(seq_wvalid && seq_address == 0)
	  seq_test <= seq_wdata;
	seq_rdata1 <= seq_rdata0;
	if(seq_rvalid)
	  case(seq_address)
	    0: seq_rdata0 <= seq_test;
	    1: seq_rdata0 <= 64'd1;
	    2: seq_rdata0 <= 64'd2;
	    3: seq_rdata0 <= 64'd3;
	    4: seq_rdata0 <= seq_spidata;
	    default: seq_rdata0 <= seq_address;
	  endcase
	
	count <= fifo_reset[6] ? 1'b0 : count + fifo_ready[6];
	count_write <= fifo_ready[6];
	
	if(fifo_ready[0])
	  led[3:0] <= fpc_data[3:0];
	fpc_read <= fifo_ready[4];
	tpc_write <= fifo_ready[0] && fpc_read;
	tpc_data <= fpc_data;
	null_read <= fifo_ready[2];
     end

   initial
     begin
	$dumpfile("dump.vcd");
	$dumpvars(0);
     end
   
endmodule
