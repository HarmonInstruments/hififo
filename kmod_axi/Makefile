obj-m := hififo.o
KERNEL_SOURCE = $(HOME)/hizynq/linux-xlnx
HOST := vna
export CROSS_COMPILE := $(HOME)/hizynq/buildroot/buildroot-2015.02/output/host/usr/bin/arm-buildroot-linux-uclibcgnueabi-

all:
	make ARCH=arm -C $(KERNEL_SOURCE) M=$(PWD) modules
clean:
	make -C $(KERNEL_SOURCE) M=$(PWD) clean
	rm -rf *~

unload:
	ssh root@$(HOST) "/sbin/rmmod hififo"
load:
	scp hififo.ko root@$(HOST):/tmp/
	ssh root@$(HOST) "/sbin/insmod /tmp/hififo.ko"

reload: unload load

dmesg:
	ssh root@$(HOST) dmesg

reload: unload load
