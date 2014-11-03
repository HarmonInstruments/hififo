all: test pyhififo.so

CCFLAGS = -c -Wall -std=gnu++11 -O3 -fPIC -fopenmp

CC = g++
HOST = vna
OBJS = TimeIt.o Sequencer.o Hififo.o Spi_Config.o
OBJS_PY = Sequencer.o Hififo.o TimeIt.o Spi_Config.o Xilinx_DRP.o pyhififo.o

pyhififo.cpp: pyhififo.pyx
	@echo Building file: $<
	cython --cplus pyhififo.pyx
	@echo ' '

pyhififo.o: pyhififo.cpp
	@echo Building file: $<
	g++ -pthread -fno-strict-aliasing -DNDEBUG -g -fwrapv -O2 -Wall -fPIC -I/usr/include/python2.7 -c pyhififo.cpp
	@echo ' '
%.o: %.cpp
	@echo Building file: $<
	$(CC) $< $(CCFLAGS)
	@echo ' '

test: test.o $(OBJS)
	@echo Building file: test
	$(CC) test.o $(OBJS) -o test -lrt -fopenmp
	@echo ' '

runtest: test
	scp test root@$(HOST):
	ssh root@$(HOST) time ./test
clean:
	rm -rf *_wrap.cxx *.o *.so *.pyc *~ hififo.py pyhififo.cpp

run: pyhififo.so
	scp ../top.bin test.py xadc.py pyhififo.so spiflash.py root@$(HOST):
	ssh root@$(HOST) "time ./test.py"

pyhififo.so: $(OBJS_PY)
	@echo Building file: pyhififo.so
	g++ -pthread -shared -Wl,-O1 -Wl,-Bsymbolic-functions -Wl,-z,relro $(OBJS_PY) -o pyhififo.so -lrt
	@echo ' '