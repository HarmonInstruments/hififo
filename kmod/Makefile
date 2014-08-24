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

test: test.c
	g++ test.c -o test -Wall -fopenmp -march=native -mavx -O3
runtest: test
	scp test root@$(HOST):
	ssh root@$(HOST) time ./test

reload: unload load

