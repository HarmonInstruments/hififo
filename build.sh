#!/bin/bash
source ~/software/Xilinx/Vivado/2014.1/settings64.sh
rm -rf xbuild/*
vivado -source init.tcl -mode tcl
cp xbuild/pcie3.runs/impl_1/xilinx_pcie_2_1_ep_7x.bin top.bin