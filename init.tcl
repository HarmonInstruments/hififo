create_project pcie3 xbuild -part xc7k70tfbg484-1
add_files -norecurse -force {hdl/}
add_files -fileset constrs_1 -norecurse -force hdl/constraints.xdc
update_compile_order -fileset sources_1
update_compile_order -fileset sim_1
create_ip -name pcie_7x -vendor xilinx.com -library ip -module_name pcie_7x_0

set_property -dict [list CONFIG.Maximum_Link_Width {X4} CONFIG.Link_Speed {5.0_GT/s} CONFIG.Bar0_Size {64} CONFIG.Use_Class_Code_Lookup_Assistant {true} CONFIG.Base_Class_Menu {Data_acquisition_and_signal_processing_controllers} CONFIG.Sub_Class_Interface_Menu {Other_data_acquisition/signal_processing_controllers} CONFIG.Max_Payload_Size {128_bytes} CONFIG.Buf_Opt_BMA {true} CONFIG.IntX_Generation {false} CONFIG.DSN_Enabled {false} CONFIG.en_ext_clk {false} CONFIG.mode_selection {Advanced}] [get_ips pcie_7x_0]
generate_target all [get_files  xbuild/pcie3.srcs/sources_1/ip/pcie_7x_0/pcie_7x_0.xci]

create_ip_run [get_files -of_objects [get_fileset sources_1] xbuild/pcie3.srcs/sources_1/ip/pcie_7x_0/pcie_7x_0.xci]
launch_run -jobs 8 pcie_7x_0_synth_1

set_property STEPS.WRITE_BITSTREAM.ARGS.BIN_FILE true [get_runs impl_1]

set_msg_config -suppress -id {filemgmt 20-1763}

launch_runs impl_1 -to_step write_bitstream
wait_on_run impl_1
exit
#open_run impl_1
#close_design
