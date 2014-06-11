`timescale 1ns / 1ps

module pcie
  (
   output [3:0]  pci_exp_txp,
   output [3:0]  pci_exp_txn,
   input [3:0] 	 pci_exp_rxp,
   input [3:0] 	 pci_exp_rxn,
   input 	 sys_clk_p,
   input 	 sys_clk_n,
   input 	 sys_rst_n,
   // FIFOs
   input 	 fifo_clock,
   output 	 fifo_reset,
   input [63:0]  fifo_to_pc_data,
   input 	 fifo_to_pc_write,
   output 	 fifo_to_pc_almost_full,
   output [63:0] fifo_from_pc_data,
   input 	 fifo_from_pc_read,
   output 	 fifo_from_pc_empty,
   // PIO
   output 	 pio_clock,
   output 	 pio_write_valid,
   output 	 pio_read_valid,
   output [63:0] pio_write_data,
   output [12:0] pio_address,
   input [63:0]  pio_read_data,
   input 	 pio_read_data_valid
   );
   
   wire 	clock;
   assign pio_clock = clock;
   assign fifo_reset = 1'b0;
   
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
   wire [23:0] 	rx_rid_tag;
      
   IBUF   pci_reset_ibuf (.O(sys_rst_n_c), .I(sys_rst_n));
   IBUFDS_GTE2 refclk_ibuf (.O(sys_clk), .ODIV2(), .I(sys_clk_p), .CEB(1'b0), .IB(sys_clk_n));

   wire 	completion_valid;
   reg 		read_done = 0;
   
   always @(posedge clock) 
     begin
	pcie_reset <= user_reset | ~user_lnk_up;
	// fix this
   	cfg_turnoff_ok <= cfg_to_turnoff; // not if a completion is pending
     end  
   
   reg [63:0] read_data = 0;
   reg [63:0] ldata = 0;

   reg [15:0] cpld_count = 0;
   reg [15:0] write_count = 0;
   reg [15:0] read_count = 0;

   wire        write_fifo_active;
   wire [19:0] write_fifo_block_count;

   wire        read_fifo_active;
   wire [17:0] read_fifo_block_count;
   
   reg [2:0]   interrupt_latch = 0;
   reg [2:0]   interrupt_enable = 0;
   wire [2:0]  interrupt;
         
   // PIO control
   always @ (posedge clock)
     begin
	if(pio_write_valid)
	  begin
	     case(pio_address)
	       0: interrupt_enable <= pio_write_data[2:0];
	     endcase
	  end
	if(completion_valid)
	  ldata <= pio_write_data;
	if((interrupt & interrupt_enable) != 0)
	  cfg_interrupt <= 1'b1;
	else if(cfg_interrupt_rdy)
	  cfg_interrupt <= 1'b0;

	cpld_count <= cpld_count + completion_valid;
	read_count <= read_count + pio_read_valid;
	write_count <= write_count + pio_write_valid;
	interrupt_latch <= (pio_read_valid && pio_address == 0) ? 1'b0 : interrupt_latch | interrupt;
	if(pio_read_valid)
	  begin
	     case(pio_address)
	       14'h0000: read_data <= {read_fifo_active, write_fifo_active, 5'd0, interrupt_latch};
	       14'h0002: read_data <= {cpld_count, write_count, read_count};
	       14'h0003: read_data <= {write_fifo_block_count, 7'd0};
	       14'h0004: read_data <= {read_fifo_block_count, 9'd0};
	       14'h0007: read_data <= ldata;
	       default: read_data <= 64'h0;
	     endcase
	  end
     end
      
   reg          read_completion_valid = 0;
   reg [23:0] 	read_completion_rid_tag = 0;
   reg [3:0] 	read_completion_lower_addr = 0;
   reg [63:0] 	read_completion_data = 0;

   always @(posedge clock)
     begin
	if(pio_read_valid)
	  begin
	     read_completion_rid_tag <= rx_rid_tag;
	     read_completion_lower_addr <= pio_address[3:0];
	  end
	read_done <= pio_read_valid;
	read_completion_valid <= read_done;
	if(read_done)
	  read_completion_data <= read_data;
     end

   wire        read_request_valid;
   wire [63:0] read_request_address;

   pcie_rx_to_fifo pcie_rx_to_fifo
     (.clock(clock),
      .reset(pcie_reset),
      // PIO control
      .pio_write_valid(pio_write_valid),
      .pio_write_address(pio_address),
      .pio_write_data(pio_write_data),
      // read completion
      .completion_valid(completion_valid),
      .completion_data(pio_write_data),
      // read request
      .read_request_valid(read_request_valid),
      .read_request_address(read_request_address),
      // status
      .fifo_interrupt_match(interrupt[1]),
      .fifo_interrupt_done(interrupt[2]),
      .fifo_active(read_fifo_active),
      .fifo_block_count(read_fifo_block_count), // [17:0] number of 512 byte blocks transferred
      // FIFO
      .fifo_clock(clock),
      .fifo_read(fifo_from_pc_read),
      .fifo_read_data(fifo_from_pc_data),
      .fifo_empty(fifo_from_pc_empty)
      );
   
   wire 	m_axis_rx_tvalid;
   wire 	m_axis_rx_tlast;
   wire [63:0] 	m_axis_rx_tdata;
   
   pcie_rx pcie_rx
     (.clock(clock),
      .reset(pcie_reset),
      // outputs
      .write_valid(pio_write_valid),
      .read_valid(pio_read_valid),
      .completion_valid(completion_valid),
      .data(pio_write_data),
      .address(pio_address),
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
      // PIO control
      .pio_write_valid(pio_write_valid),
      .pio_write_address(pio_address),
      .pio_write_data(pio_write_data),
      // read completion
      .read_completion_valid(read_completion_valid),
      .read_completion_rid_tag(read_completion_rid_tag), // 24
      .read_completion_lower_addr(read_completion_lower_addr),// 4
      .read_completion_data(read_completion_data),
      // status
      .fifo_interrupt_match(interrupt[0]),
      .fifo_active(write_fifo_active),
      .fifo_block_count(write_fifo_block_count), // [19:0] number of 128 byte blocks transmitted
      // FIFO
      .fifo_clock(clock),
      .fifo_write_valid(fifo_to_pc_write),
      .fifo_write_data(fifo_to_pc_data),
      .fifo_almost_full(fifo_to_pc_almost_full),
      // read request
      .read_request_valid(read_request_valid),
      .read_request_address(read_request_address),
      .read_request_tag(8'h0),
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

