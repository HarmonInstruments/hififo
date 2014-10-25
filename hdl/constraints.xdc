# HIFIFO: Harmon Instruments PCI Express to FIFO
# Copyright (C) 2014 Harmon Instruments, LLC
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/

set_property PULLUP true [get_ports pcie_rst_n]
set_property IOSTANDARD LVCMOS18 [get_ports pcie_rst_n]
set_property PACKAGE_PIN B11 [get_ports pcie_rst_n]
set_property LOC IBUFDS_GTE2_X0Y1 [get_cells hififo/pcie_core_wrap/refclk_ibuf]
create_clock -period 10.000 -name sys_clk [get_pins hififo/pcie_core_wrap/refclk_ibuf/O]
set_false_path -from [get_ports pcie_rst_n]

# SPI flash
set_property PULLUP true [get_ports cflash*]
set_property IOSTANDARD LVCMOS18 [get_ports cflash*]
set_property DRIVE 4 [get_ports cflash*]
set_property SLEW SLOW [get_ports cflash*]
set_property PACKAGE_PIN L16 [get_ports cflash_cs]
#set_property PACKAGE_PIN G7  [get_ports cflash_sck]
set_property PACKAGE_PIN H18 [get_ports cflash_sdi]
set_property PACKAGE_PIN H19 [get_ports cflash_sdo]
set_property PACKAGE_PIN G18 [get_ports cflash_high[0]]
set_property PACKAGE_PIN F19 [get_ports cflash_high[1]]

set_property CONFIG_MODE SPIx4 [current_design]
set_property BITSTREAM.GENERAL.COMPRESS TRUE [current_design]
set_property BITSTREAM.CONFIG.CONFIGRATE 33 [current_design]
set_property BITSTREAM.CONFIG.USERID 0XDEADBEEF [current_design]
set_property CONFIG_VOLTAGE 1.8 [current_design]
set_property BITSTREAM.CONFIG.SPI_BUSWIDTH 4 [current_design]
set_property CFGBVS GND [current_design]

set_property IOSTANDARD LVCMOS18 [get_ports led*]
set_property PACKAGE_PIN A11 [get_ports {led[0]}]
set_property PACKAGE_PIN A10 [get_ports {led[1]}]
set_property PACKAGE_PIN A9  [get_ports {led[2]}]
set_property PACKAGE_PIN A8  [get_ports {led[3]}]
set_property DRIVE 4 [get_ports led*]
set_property SLEW SLOW [get_ports led*]
