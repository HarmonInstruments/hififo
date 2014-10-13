all: test pyhififo.so

CCFLAGS = -c -Wall -std=gnu++11 -O3 -fPIC

CC = g++
HOST = vna
OBJS = TimeIt.o AlignedMem.o Sequencer.o Hififo.o Spi_Config.o
OBJS_PY = Sequencer.o Hififo.o TimeIt.o AlignedMem.o Spi_Config.o Xilinx_DRP.o pyhififo.o

pyhififo.cpp: pyhififo.pyx
	cython --cplus pyhififo.pyx
pyhififo.o: pyhififo.cpp
	g++ -pthread -fno-strict-aliasing -DNDEBUG -g -fwrapv -O2 -Wall -fPIC -I/usr/include/python2.7 -c pyhififo.cpp

%.o: %.cpp
	@echo Building file: $<
	$(CC) $< $(CCFLAGS)
	@echo ' '

test: test.o $(OBJS)
	$(CC) test.o $(OBJS) -o test -lrt
runtest: test
	scp test root@$(HOST):
	ssh root@$(HOST) time ./test
clean:
	rm -rf *_wrap.cxx *.o *.so *.pyc *~ hififo.py pyhififo.cpp

run: pyhififo.so
	scp ../top.bin test.py xadc.py pyhififo.so spiflash.py root@$(HOST):
	ssh root@$(HOST) "time ./test.py"

pyhififo.so: $(OBJS_PY)
	g++ -pthread -shared -Wl,-O1 -Wl,-Bsymbolic-functions -Wl,-z,relro $(OBJS_PY) -o pyhififo.so -lrt
