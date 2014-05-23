top.bin: build.sh build.tcl hdl
	./build.sh
load:
	./jtag.py xbuild/pcie3.runs/impl_1/vna_dsp.bin
	ssh root@vna reboot

clean: semiclean
	rm -rf *.bin
semiclean:
	rm -rf xbuild *~ hdl/*~ hdl/testbenches/*~ *.log *.jou