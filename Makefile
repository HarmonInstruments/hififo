obj-m := vna_dsp.o

KERNEL_SOURCE = /lib/modules/$(shell uname -r)/build

all:
	make -C $(KERNEL_SOURCE) M=$(PWD) modules
clean:
	make -C $(KERNEL_SOURCE) M=$(PWD) clean
	rm -rf *~
unload:
	sudo rmmod vna_dsp
load:
	sudo insmod vna_dsp.ko
reload: unload load