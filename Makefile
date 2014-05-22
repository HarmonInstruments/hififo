top.bin: build.sh build.tcl hdl
	./build.sh
load:
	./jtag.py top.bin 

clean: semiclean
	rm -rf *.bin
semiclean:
	rm -rf xbuild *~ hdl/*~ hdl/testbenches/*~ *.log *.jou