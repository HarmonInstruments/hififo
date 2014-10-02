all: test hififo.py

CCFLAGS = -c -Wall -std=gnu++11 -O2 -fPIC
LDFLAGS = -Wall -std=gnu++11 -fopenmp -march=native -mavx -O0 -fPIC
CC = g++
HOST = vna
OBJS = TimeIt.o AlignedMem.o Sequencer.o Hififo.o

%.o: %.cpp
	@echo Building file: $<
	$(CC) $< $(CCFLAGS)
	@echo ' '

test: test.o $(OBJS)
	$(CC) test.o $(OBJS) -o test -lrt
runtest: test
	scp test root@$(HOST):
	ssh root@$(HOST) time ./test

hififo.py: hififo.i $(OBJS)
	swig -python -c++ hififo.i
	$(CC) $(CCFLAGS) hififo_wrap.cxx -I/usr/include/python2.7
	$(CC) -shared hififo_wrap.o $(OBJS) -o _hififo.so -lrt
clean:
	rm -rf *_wrap.cxx *.o *.so *.pyc *~ hififo.py