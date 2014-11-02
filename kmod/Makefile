obj-m := hififo.o
KERNEL_SOURCE = /lib/modules/$(shell uname -r)/build
HOST := vna
all:
	make -C $(KERNEL_SOURCE) M=$(PWD) modules
clean:
	make -C $(KERNEL_SOURCE) M=$(PWD) clean
	rm -rf *~

unload:
	ssh root@$(HOST) "rmmod hififo"
load:
	ssh $(HOST) "rm -rf kmod; mkdir kmod"
	scp Makefile hififo.c vna:kmod
	ssh $(HOST) "cd kmod; make clean all"
	ssh root@$(HOST) "insmod /home/dlharmon/kmod/hififo.ko"
dmesg:
	ssh $(HOST) dmesg

reload: unload load

