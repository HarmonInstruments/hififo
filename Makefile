obj-m := vna_dsp.o

KERNEL_SOURCE = /lib/modules/$(shell uname -r)/build

all:
	make -C $(KERNEL_SOURCE) M=$(PWD) modules
clean:
	make -C $(KERNEL_SOURCE) M=$(PWD) clean
	rm -rf *~
unload:
	ssh root@vna "rmmod vna_dsp"
load: 
	ssh vna "rm -rf kmod; mkdir kmod"
	scp Makefile vna_dsp.c vna:kmod
	ssh vna "cd kmod; make clean all"
	ssh root@vna "insmod /home/dlharmon/kmod/vna_dsp.ko"

dmesg:
	ssh vna dmesg

reload: unload load