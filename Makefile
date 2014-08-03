top.bin: build.sh init.tcl hdl
	./build.sh
load_fpga:
	./jtag.py xbuild/pcie3.runs/impl_1/vna_dsp.bin
	ssh root@vna reboot
clean: semiclean
	rm -rf *.bin
semiclean:
	rm -rf xbuild *~ hdl/*~ hdl/testbenches/*~ *.log *.jou
	make -C kmod clean
unload_kmod:
	make -C kmod unload
load_kmod: 
	make -C kmod load
dmesg:
	make -C kmod dmesg
runtest:
	make -C kmod runtest
reload:
	make -C kmod reload
