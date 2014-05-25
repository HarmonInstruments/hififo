`timescale 1ns / 1ps

module pcie
  (
   output [3:0]     pci_exp_txp,
   output [3:0]     pci_exp_txn,
   input [3:0] 	    pci_exp_rxp,
   input [3:0] 	    pci_exp_rxn,
   input 	    sys_clk_p,
   input 	    sys_clk_n,
   input 	    sys_rst_n,
/*   output 	    pio_write_valid,
   output 	    pio_read_valid,
   output [63:0]    pio_write_data,
   output [12:0]    pio_address,
   input [63:0]     pio_read_data,*/
   output reg [3:0] led = 4'h5
   );
   
   wire 	clock;
   wire 	user_reset;
   wire 	user_lnk_up;
   wire 	cfg_to_turnoff;
   wire [15:0] 	pcie_id;
   wire 	sys_rst_n_c;
   wire 	sys_clk;
   reg 		pcie_reset = 1'b1;
   reg 		cfg_turnoff_ok = 0;
   wire 	cfg_interrupt_rdy;
   reg 		cfg_interrupt = 0;
      
   IBUF   pci_reset_ibuf (.O(sys_rst_n_c), .I(sys_rst_n));
   IBUFDS_GTE2 refclk_ibuf (.O(sys_clk), .ODIV2(), .I(sys_clk_p), .CEB(1'b0), .IB(sys_clk_n));

   wire [23:0] 	rx_rid_tag;
   wire [12:0] 	rx_address;
   wire 	read_valid;
   wire 	write_valid;
   wire 	completion_valid;
   wire [63:0] 	write_data;
   reg 		read_done = 0;
   
   always @(posedge clock) 
     begin
	pcie_reset <= user_reset | ~user_lnk_up;
	// fix this
   	cfg_turnoff_ok <= cfg_to_turnoff; // not if a completion is pending
	if(write_valid)
	  led[3:0] <= write_data[3:0];
     end  
   
   reg [63:0] read_data = 0;
   reg [63:0] ldata = 0;

   reg [15:0] cpld_count = 0;
   reg [15:0] write_count = 0;
   reg [15:0] read_count = 0;

   wire        write_fifo_active;
   wire        write_fifo_interrupt_match;
   wire        write_fifo_interrupt_fifo;
   wire [23:0] write_fifo_block_count;
   wire        write_fifo_ready;

   // read data
   always @ (posedge clock)
     begin
	if(write_valid)
	  begin
	     if(rx_address == 13'h0000)
	       ldata <= write_data;
	  end
	else if(completion_valid)
	  ldata <= write_data;
	if(write_valid && (rx_address == 102))
	  cfg_interrupt <= 1'b1;
	else if(cfg_interrupt_rdy)
	  cfg_interrupt <= 1'b0;
	
	read_done <= read_valid;
	cpld_count <= cpld_count + completion_valid;
	read_count <= read_count + read_valid;
	write_count <= write_count + write_valid;
	if(read_valid)
	  begin
	     case(rx_address)
	       14'h0000: read_data <= ldata;
	       14'h0001: read_data <= cpld_count;
	       14'h0002: read_data <= write_count;
	       14'h0003: read_data <= read_count;
	       14'h0004: read_data <= {write_fifo_block_count, 7'd0};
	       14'h0005: read_data <= {write_fifo_active, write_fifo_interrupt_match, write_fifo_interrupt_fifo, write_fifo_ready};
	       default: read_data <= {rx_address, 32'hDEADBEEF};
	     endcase
	  end
     end
      
   reg          read_completion_valid = 0;
   reg [23:0] 	read_completion_rid_tag = 0;
   reg [3:0] 	read_completion_lower_addr = 0;
   reg [63:0] 	read_completion_data = 0;
   wire 	read_completion_ready;

   always @(posedge clock)
     begin
	if(read_done)
	  begin
	     read_completion_valid <= 1'b1;
	     read_completion_rid_tag <= rx_rid_tag;
	     read_completion_lower_addr <= rx_address[3:0];
	     read_completion_data <= read_data;
	  end
	else if(read_completion_ready)
	  begin
	     read_completion_valid <= 1'b0;
	  end
     end // always @ (posedge clock)

   wire        write_request_valid;
   wire [63:0] write_request_data;
   wire [63:0] write_request_address;
   wire        write_request_ready;
   wire        write_request_ack;
   
   pcie_dma_write_fifo pcie_dma_write_fifo
     (.clock(clock),
      .reset(pcie_reset),
      // PIO control
      .pio_write_valid(write_valid),
      .pio_write_address(rx_address),
      .pio_write_data(write_data),
      // status
      .interrupt_match(write_fifo_interrupt_match),
      .interrupt_fifo(write_fifo_interrupt_fifo),
      .active(write_fifo_active),
      .block_count(write_fifo_block_count), // [23:0] number of 128 byte blocks transmitted
      // FIFO
      .fifo_clock(clock),
      .fifo_write_valid(write_valid && {rx_address[12:1], 1'b0} == 16),
      .fifo_write_data({rx_address[0], write_data}), // bit 64 is end, 65 is interrupt
      .fifo_ready(write_fifo_ready), // minimum 32 positions available
      // interface to pcie_tx module
      .write_request_valid(write_request_valid),
      .write_request_data(write_request_data),
      .write_request_address(write_request_address),
      .write_request_ready(write_request_ready), // pulses 16 times to read request data
      .write_request_ack(write_request_ack) // pulses high once to indicate the header is complete
      );
     
   reg        read_request_valid = 0;
   reg [63:0] read_request_address = 0;
   wire       read_request_ready;

   always @ (posedge clock)
     begin
	if(write_valid && (rx_address == 101))
	  begin
	     read_request_valid <= 1'b1;
	     read_request_address <= write_data;
	  end
	else if(read_request_ready)
	  read_request_valid <= 1'b0;
     end

   wire 	m_axis_rx_tvalid;
   wire 	m_axis_rx_tlast;
   wire [63:0] 	m_axis_rx_tdata;
   
   pcie_rx pcie_rx
     (.clock(clock),
      .reset(pcie_reset),
      // outputs
      .write_valid(write_valid),
      .read_valid(read_valid),
      .completion_valid(completion_valid),
      .data(write_data),
      .address(rx_address),
      .rid_tag(rx_rid_tag),
      // AXI stream from PCIE core
      .tvalid(m_axis_rx_tvalid),
      .tlast(m_axis_rx_tlast),
      .tdata(m_axis_rx_tdata)
      );		     
   
   wire 	s_axis_tx_tready;
   wire [63:0] 	s_axis_tx_tdata;
   wire [7:0] 	s_axis_tx_tkeep;
   wire 	s_axis_tx_tlast;
   wire 	s_axis_tx_tvalid;
   
   pcie_tx pcie_tx
     (.clock(clock),
      .reset(pcie_reset),
      .pcie_id(pcie_id),
      // read completion
      .read_completion_valid(read_completion_valid),
      .read_completion_rid_tag(read_completion_rid_tag), // 24
      .read_completion_lower_addr(read_completion_lower_addr),// 4
      .read_completion_data(read_completion_data),
      .read_completion_ready(read_completion_ready),
      // write
      .write_request_valid(write_request_valid),
      .write_request_data(write_request_data),
      .write_request_address(write_request_address),
      .write_request_ready(write_request_ready),
      .write_request_ack(write_request_ack),
      // read request
      .read_request_valid(read_request_valid),
      .read_request_address(read_request_address),
      .read_request_tag(8'h0),
      .read_request_ready(read_request_ready),
      // AXI stream to PCI Express core
      .axis_tx_tready(s_axis_tx_tready),
      .axis_tx_tdata(s_axis_tx_tdata),
      .axis_tx_1dw(s_axis_tx_1dw),
      .axis_tx_tlast(s_axis_tx_tlast),
      .axis_tx_tvalid(s_axis_tx_tvalid)
   );
   
   pcie_7x_0 pcie_7x_0_i
     (
      // PCI express data pairs
      .pci_exp_txn(pci_exp_txn), .pci_exp_txp(pci_exp_txp),
      .pci_exp_rxn(pci_exp_rxn), .pci_exp_rxp(pci_exp_rxp),
      // AXI
      .user_clk_out(clock),
      .user_reset_out(user_reset),
      .user_lnk_up(user_lnk_up),
      .user_app_rdy(),
      .s_axis_tx_tready(s_axis_tx_tready),
      .s_axis_tx_tdata(s_axis_tx_tdata),
      .s_axis_tx_tkeep(s_axis_tx_1dw ? 8'h0F : 8'hFF),
      .s_axis_tx_tuser(4'd0), // may want to assert 2 for cut through
      .s_axis_tx_tlast(s_axis_tx_tlast),
      .s_axis_tx_tvalid(s_axis_tx_tvalid),
      .m_axis_rx_tdata(m_axis_rx_tdata),
      .m_axis_rx_tkeep(),
      .m_axis_rx_tlast(m_axis_rx_tlast),
      .m_axis_rx_tvalid(m_axis_rx_tvalid),
      .m_axis_rx_tready(1'b1), // always ready
      .m_axis_rx_tuser(),
      
      .tx_cfg_gnt(1'b1), .rx_np_ok(1'b1), .rx_np_req(1'b1),
      .cfg_trn_pending(1'b0),
      .cfg_pm_halt_aspm_l0s(1'b0), .cfg_pm_halt_aspm_l1(1'b0),
      .cfg_pm_force_state_en(1'b0), .cfg_pm_force_state(2'd0),
      .cfg_dsn(64'h0),
      .cfg_turnoff_ok(cfg_turnoff_ok),
      .cfg_pm_wake(1'b0), .cfg_pm_send_pme_to(1'b0),
      .cfg_ds_bus_number(8'b0),
      .cfg_ds_device_number(5'b0),
      .cfg_ds_function_number(3'b0),
      // flow control
      .fc_cpld(), .fc_cplh(), .fc_npd(), .fc_nph(),
      .fc_pd(), .fc_ph(), .fc_sel(3'd0),
      // configuration
      .cfg_dcommand2(), .cfg_pmcsr_pme_status(),
      .cfg_status(), .cfg_to_turnoff(cfg_to_turnoff),
      .cfg_received_func_lvl_rst(),
      .cfg_dcommand(),
      .cfg_bus_number(pcie_id[15:8]),
      .cfg_device_number(pcie_id[7:3]),      
      .cfg_function_number(pcie_id[2:0]),
      .cfg_command(),
      .cfg_dstatus(),
      .cfg_lstatus(),
      .cfg_pcie_link_state(),
      .cfg_lcommand(),
      .cfg_pmcsr_pme_en(),
      .cfg_pmcsr_powerstate(),
      .tx_buf_av(),
      .tx_err_drop(),
      .tx_cfg_req(),
      // root port only
      .cfg_bridge_serr_en(),
      .cfg_slot_control_electromech_il_ctl_pulse(),
      .cfg_root_control_syserr_corr_err_en(),
      .cfg_root_control_syserr_non_fatal_err_en(),
      .cfg_root_control_syserr_fatal_err_en(),
      .cfg_root_control_pme_int_en(),
      .cfg_aer_rooterr_corr_err_reporting_en(),
      .cfg_aer_rooterr_non_fatal_err_reporting_en(),
      .cfg_aer_rooterr_fatal_err_reporting_en(),
      .cfg_aer_rooterr_corr_err_received(),
      .cfg_aer_rooterr_non_fatal_err_received(),
      .cfg_aer_rooterr_fatal_err_received(),
      // both
      .cfg_vc_tcvc_map(),
      // Management Interface
      .cfg_mgmt_di(32'h0),
      .cfg_mgmt_byte_en(4'h0),
      .cfg_mgmt_dwaddr(10'h0),
      .cfg_mgmt_wr_en(1'b0),
      .cfg_mgmt_rd_en(1'b0),
      .cfg_mgmt_wr_readonly(1'b0),
      .cfg_mgmt_wr_rw1c_as_rw(1'b0),
      .cfg_mgmt_do(),
      .cfg_mgmt_rd_wr_done(),
      // Error Reporting Interface
      .cfg_err_ecrc(1'b0),
      .cfg_err_ur(1'b0),
      .cfg_err_cpl_timeout(1'b0),
      .cfg_err_cpl_unexpect(1'b0),
      .cfg_err_cpl_abort(1'b0),
      .cfg_err_posted(1'b0),
      .cfg_err_cor(1'b0),
      .cfg_err_atomic_egress_blocked(1'b0),
      .cfg_err_internal_cor(1'b0),
      .cfg_err_malformed(1'b0),
      .cfg_err_mc_blocked(1'b0),
      .cfg_err_poisoned(1'b0),
      .cfg_err_norecovery(1'b0),
      .cfg_err_tlp_cpl_header(48'h0),
      .cfg_err_cpl_rdy(),
      .cfg_err_locked(1'b0),
      .cfg_err_acs(1'b0),
      .cfg_err_internal_uncor(1'b0),
      
      .cfg_err_aer_headerlog(128'h0), .cfg_aer_interrupt_msgnum(5'h0),
      .cfg_err_aer_headerlog_set(), .cfg_aer_ecrc_check_en(), .cfg_aer_ecrc_gen_en(),
      
      .cfg_interrupt(cfg_interrupt),
      .cfg_interrupt_rdy(cfg_interrupt_rdy),
      .cfg_interrupt_assert(1'b0),
      .cfg_interrupt_di(8'h0),
      .cfg_interrupt_do(),
      .cfg_interrupt_mmenable(),
      .cfg_interrupt_msienable(),
      .cfg_interrupt_msixenable(),
      .cfg_interrupt_msixfm(),
      .cfg_interrupt_stat(1'b0),
      .cfg_pciecap_interrupt_msgnum(5'h0),      
      .cfg_msg_received_err_cor(),
      .cfg_msg_received_err_non_fatal(),
      .cfg_msg_received_err_fatal(),
      .cfg_msg_received_pm_as_nak(),
      .cfg_msg_received_pme_to_ack(),
      .cfg_msg_received_assert_int_a(), .cfg_msg_received_assert_int_b(),
      .cfg_msg_received_assert_int_c(), .cfg_msg_received_assert_int_d(),
      .cfg_msg_received_deassert_int_a(), .cfg_msg_received_deassert_int_b(),
      .cfg_msg_received_deassert_int_c(), .cfg_msg_received_deassert_int_d(),
      .cfg_msg_received_pm_pme(),
      .cfg_msg_received_setslotpowerlimit(),
      .cfg_msg_received(),
      .cfg_msg_data(),
      .pl_directed_link_change(2'd0), .pl_directed_link_width(2'd0),
      .pl_directed_link_speed(1'b0), .pl_directed_link_auton(1'b0),
      .pl_upstream_prefer_deemph(1'b1), .pl_sel_lnk_rate(), .pl_sel_lnk_width(),
      .pl_ltssm_state(), .pl_lane_reversal_mode(),
      .pl_phy_lnk_up(), .pl_tx_pm_state(), .pl_rx_pm_state(), .pl_link_upcfg_cap(),
      .pl_link_gen2_cap(), .pl_link_partner_gen2_supported(),
      .pl_initial_link_width(), .pl_directed_change_done(),
      .pl_received_hot_rst(), .pl_transmit_hot_rst(1'b0),
      .pl_downstream_deemph_source(1'b0),
      .pcie_drp_clk(1'b1), .pcie_drp_en(1'b0), .pcie_drp_we(1'b0),
      .pcie_drp_addr(9'h0), .pcie_drp_di(16'h0),
      .pcie_drp_rdy(), .pcie_drp_do(),
      // these are the clock and reset from the card edge connector
      .sys_clk                                    ( sys_clk ),
      .sys_rst_n                                  ( sys_rst_n_c )
      );      
endmodule

