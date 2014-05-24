obj-m := vna_dsp.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
	rm -rf *~
unload:
	sudo rmmod vna_dsp
load:
	sudo insmod vna_dsp.ko
reload: unload load