#!/bin/bash
source ~/software/Xilinx/Vivado/2014.3/settings64.sh
./timestamp.sh
rm -rf xbuild/*
vivado -source init.tcl -mode tcl
cp xbuild/hififo.runs/impl_1/vna_dsp.bin top.bin
