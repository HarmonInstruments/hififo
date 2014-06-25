set_property PULLUP true [get_ports pcie_rst_n]
set_property IOSTANDARD LVCMOS18 [get_ports pcie_rst_n]
set_property PACKAGE_PIN B11 [get_ports pcie_rst_n]
set_property LOC IBUFDS_GTE2_X0Y1 [get_cells pcie_core_wrap/refclk_ibuf]
create_clock -period 10.000 -name sys_clk [get_pins pcie_core_wrap/refclk_ibuf/O]
set_false_path -from [get_ports pcie_rst_n]

set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 3 [current_design]
set_property CONFIG_VOLTAGE 1.8 [current_design]
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 1 [current_design]
set_property CFGBVS GND [current_design]

set_property IOSTANDARD LVCMOS18 [get_ports led*]
set_property PACKAGE_PIN A11 [get_ports {led[0]}]
set_property PACKAGE_PIN A10 [get_ports {led[1]}]
set_property PACKAGE_PIN A9  [get_ports {led[2]}]
set_property PACKAGE_PIN A8  [get_ports {led[3]}]
set_property DRIVE 4 [get_ports led*]
set_property SLEW SLOW [get_ports led*]

   output [3:0]     pcie_txp,
   output [3:0]     pcie_txn,
   input [3:0]      pcie_rxp,
   input [3:0]      pcie_rxn,
   input            pcie_refclk_p,
   input            pcie_refclk_n,
   input            pcie_rst_n,
   output reg [3:0] led = 4'h5
