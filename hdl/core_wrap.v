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

`ifndef SIM

module pcie_core_wrap
  (
   // IO pins
   output [NLANES-1:0] 	  pci_exp_txp,
   output [NLANES-1:0] 	  pci_exp_txn,
   input [NLANES-1:0] 	  pci_exp_rxp,
   input [NLANES-1:0] 	  pci_exp_rxn,
   input 		  sys_clk_p,
   input 		  sys_clk_n,
   input 		  sys_rst_n,
   //
   output [15:0] 	  pci_id,
   input 		  interrupt,
   output 		  interrupt_rdy,
   output reg 		  pci_reset = 1,
   output 		  clock,
   // DRP
   input [9*NLANES-1:0]   gt_drp_address,
   input [NLANES-1:0] 	  gt_drp_en,
   input [16*NLANES-1:0]  gt_drp_di,
   output [16*NLANES-1:0] gt_drp_do,
   output [NLANES-1:0] 	  gt_drp_ready,
   input [NLANES-1:0] 	  gt_drp_we,
   input 		  gt_drp_clock,
   // AXI to core
   output 		  s_axis_tx_tready,
   input [63:0] 	  s_axis_tx_tdata,
   input 		  s_axis_tx_1dw,
   input 		  s_axis_tx_tlast,
   input 		  s_axis_tx_tvalid,
   // AXI from core
   output 		  m_axis_rx_tvalid,
   output 		  m_axis_rx_tlast,
   output [63:0] 	  m_axis_rx_tdata
   );

   parameter NLANES = 4;

   wire 	user_reset;
   wire 	user_lnk_up;
   wire 	cfg_to_turnoff;
   wire 	sys_rst_n_c;
   wire 	sys_clk;
   reg 		cfg_turnoff_ok = 0;
   always @(posedge clock)
     begin
	pci_reset <= user_reset | ~user_lnk_up;
	// fix this
   	cfg_turnoff_ok <= cfg_to_turnoff; // not if a completion is pending
     end

   IBUF   pci_reset_ibuf (.O(sys_rst_n_c), .I(sys_rst_n));
   IBUFDS_GTE2 refclk_ibuf (.O(sys_clk), .ODIV2(), .I(sys_clk_p), .CEB(1'b0), .IB(sys_clk_n));

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
      .cfg_bus_number(pci_id[15:8]),
      .cfg_device_number(pci_id[7:3]),
      .cfg_function_number(pci_id[2:0]),
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

      .cfg_interrupt(interrupt),
      .cfg_interrupt_rdy(interrupt_rdy),
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
      // DRP
      .ext_ch_gt_drpaddr(gt_drp_address),
      .ext_ch_gt_drpen(gt_drp_en),
      .ext_ch_gt_drpdi(gt_drp_di),
      .ext_ch_gt_drpdo(gt_drp_do),
      .ext_ch_gt_drprdy(gt_drp_ready),
      .ext_ch_gt_drpwe(gt_drp_we),
      .ext_ch_gt_drpclk(gt_drp_clock),
      // these are the clock and reset from the card edge connector
      .sys_clk                                    ( sys_clk ),
      .sys_rst_n                                  ( sys_rst_n_c )
      );
endmodule

`endif

`ifdef SIM
module pcie_core_wrap
  (
   // IO pins
   output [NLANES-1:0] 	  pci_exp_txp,
   output [NLANES-1:0] 	  pci_exp_txn,
   input [NLANES-1:0] 	  pci_exp_rxp,
   input [NLANES-1:0] 	  pci_exp_rxn,
   input 		  sys_clk_p,
   input 		  sys_clk_n,
   input 		  sys_rst_n,
   output reg [15:0] 	  pci_id = 16'hDEAD,
   input 		  interrupt,
   output reg 		  interrupt_rdy = 0,
   output reg 		  pci_reset = 0,
   output reg 		  clock = 0,
   // DRP
   input [9*NLANES-1:0]   gt_drp_address,
   input [NLANES-1:0] 	  gt_drp_en,
   input [16*NLANES-1:0]  gt_drp_di,
   output [16*NLANES-1:0] gt_drp_do,
   output [NLANES-1:0] 	  gt_drp_ready,
   input [NLANES-1:0] 	  gt_drp_we,
   input 		  gt_drp_clock,
   // AXI to core
   output reg 		  s_axis_tx_tready = 0,
   input [63:0] 	  s_axis_tx_tdata,
   input 		  s_axis_tx_1dw,
   input 		  s_axis_tx_tlast,
   input 		  s_axis_tx_tvalid,
   // AXI from core
   output reg 		  m_axis_rx_tvalid = 0,
   output reg 		  m_axis_rx_tlast = 0,
   output reg [63:0] 	  m_axis_rx_tdata = 0
   );

   parameter NLANES = 4;

   always @ (posedge clock)
     interrupt_rdy <= interrupt;
endmodule

`endif