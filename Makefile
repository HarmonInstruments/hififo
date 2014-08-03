top.bin: build.sh init.tcl hdl
	./build.sh
load_fpga:
	./jtag.py xbuild/pcie3.runs/impl_1/vna_dsp.bin
	ssh root@vna reboot

clean: semiclean
	rm -rf *.bin
semiclean:
	rm -rf xbuild *~ hdl/*~ hdl/testbenches/*~ *.log *.jou

obj-m := vna_dsp.o

KERNEL_SOURCE = /lib/modules/$(shell uname -r)/build

all:
	make -C $(KERNEL_SOURCE) M=$(PWD) modules
clean:
	make -C $(KERNEL_SOURCE) M=$(PWD) clean
	rm -rf *~
unload_kmod:
	ssh root@vna "rmmod vna_dsp"
load_kmod: 
	ssh vna "rm -rf kmod; mkdir kmod"
	scp Makefile vna_dsp.c vna:kmod
	ssh vna "cd kmod; make clean all"
	ssh root@vna "insmod /home/dlharmon/kmod/vna_dsp.ko"

dmesg:
	ssh vna dmesg

test: test.c
	gcc test.c -o test -std=c99 -Wall -fopenmp -march=native -mavx -O3
runtest: test
	scp test root@vna:
	ssh root@vna time ./test

reload: unload load

