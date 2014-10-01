export VERILOG_SOURCES = $(PWD)/../hififo.v \
	$(PWD)/../pcie_tx.v \
	$(PWD)/../pcie_rx.v \
	$(PWD)/../sync.v \
	$(PWD)/../fifo.v \
	$(PWD)/../hififo_fpc_fifo.v \
	$(PWD)/../hififo_tpc_fifo.v \
	$(PWD)/../hififo_fetch_descriptor.v \
	$(PWD)/../block_ram.v \
	$(PWD)/../core_wrap.v \
	$(PWD)/../top.v \
	$(PWD)/../sequencer.v \
	$(PWD)/../xadc.v \
	$(PWD)/../spi_8.v

export TOPLEVEL = vna_dsp
export COCOTB=/home/dlharmon/software/cocotb
export MODULE=test
export COMPILE_ARGS=-D SIM

test:
	make -C coco

clean:
	rm -rf *.pyc *~
	rm -rf coco/*~
	rm -rf coco/*.vcd
	rm -rf coco/*.xml
	make -C coco clean