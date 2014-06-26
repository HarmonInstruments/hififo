module tb_hififo_pcie;
   reg clock = 0;
   reg reset = 1;
   wire interrupt;
   
   // AXI
   reg 	t_tready;
   wire [63:0] t_tdata;
   wire        t_1dw;
   wire        t_tlast;
   wire        t_tvalid;
   reg 	       r_tvalid;
   reg 	       r_tlast;
   reg [63:0]  r_tdata;

   reg [63:0]  tpc0_data;
   reg 	       tpc0_write;
   wire        tpc0_ready;

   wire [63:0] fpc0_data;
   reg 	       fpc0_read;
   wire        fpc0_empty;
     
   initial begin
      $dumpfile("dump.vcd");
      $dumpvars(0, dut);
      $from_myhdl(clock, reset, t_tready, r_tvalid, r_tlast, r_tdata, tpc0_data, tpc0_write, fpc0_read);
      $to_myhdl(interrupt, t_tdata, t_1dw, t_tlast, t_tvalid, tpc0_ready, fpc0_data, fpc0_empty);
   end
   
   hififo_pcie dut
     (.clock(clock),
      .pci_reset(reset),
      .pci_id(16'hDEAD),
      .interrupt_out(interrupt),
      .s_axis_tx_tready(t_tready),
      .s_axis_tx_tdata(t_tdata),
      .s_axis_tx_1dw(t_1dw),
      .s_axis_tx_tlast(t_tlast),
      .s_axis_tx_tvalid(t_tvalid),
      // AXI from core
      .m_axis_rx_tvalid(r_tvalid),
      .m_axis_rx_tlast(r_tlast),
      .m_axis_rx_tdata(r_tdata),
      // FIFOs
      .fifo_clock(clock),
      .tpc0_reset(),
      .tpc0_data(tpc0_data),
      .tpc0_write(tpc0_write),
      .tpc0_ready(tpc0_ready),
      .fpc0_reset(),
      .fpc0_data(fpc0_data),
      .fpc0_read(fpc0_read),
      .fpc0_empty(fpc0_empty),
      // PIO
      .pio_write_valid(),
      .pio_write_data(),
      .pio_address()
      );
   
endmodule
