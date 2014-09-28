TOPLEVEL = vna_dsp
COCOTB=/home/dlharmon/software/cocotb
VERILOG_SOURCES = $(PWD)/../hififo.v $(PWD)/../pcie_tx.v $(PWD)/../pcie_rx.v $(PWD)/../sync.v $(PWD)/../fifo.v $(PWD)/../hififo_fpc_fifo.v $(PWD)/../hififo_tpc_fifo.v $(PWD)/../hififo_fetch_descriptor.v $(PWD)/../block_ram.v $(PWD)/../core_wrap.v $(PWD)/../top.v $(PWD)/../sequencer.v 
MODULE=test
COMPILE_ARGS=-D SIM
include $(COCOTB)/makefiles/Makefile.inc
include $(COCOTB)/makefiles/Makefile.sim