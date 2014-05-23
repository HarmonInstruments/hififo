obj-m := pciesdr.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
	rm -rf *~
unload:
	sudo rmmod pciesdr
load:
	sudo insmod pciesdr.ko
reload: unload load