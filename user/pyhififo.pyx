from cython.operator cimport dereference as deref

ctypedef unsigned long long uint64_t
ctypedef unsigned long uint32_t

cdef extern from "Sequencer.h":# namespace "":
    cdef cppclass Sequencer:
        Sequencer(char *, char *) except +
        void append(uint64_t data)
        void wait(uint64_t count)
        void write_req(size_t count, uint32_t address, uint64_t *data)
        void write_single(uint32_t address, uint64_t data)
        void read_req(size_t count, uint32_t address)
        uint64_t * run()
        void write(uint32_t address, uint64_t data)
        uint64_t read(uint32_t address)

cdef class PySequencer:
    cdef Sequencer *thisptr      # hold a C++ instance which we're wrapping
    def __cinit__(self, char* name_write, char* name_read):
        self.thisptr = new Sequencer(name_write, name_read)
    def __dealloc__(self):
        del self.thisptr
    def append(self, uint64_t count):
        self.thisptr.wait(count)
        return
#def write_req(self, size_t count, uint32_t address, uint64_t *data):
#        self.thisptr.write_req(count, address, data)
     #   return
    def write_single(self, uint32_t address, uint64_t data):
        self.thisptr.write_single(address, data)
        return
    def read_req(self, size_t count, uint32_t address):
        self.thisptr.read_req(count, address)
        return
    def write(self, uint32_t address, uint64_t data):
        self.thisptr.write(address, data)
        return
    def read(self, uint32_t address):
        return self.thisptr.read(address)

#    def run(self):
#        return self.thisptr.run(dx, dy)

cdef extern from "Spi_Config.h":
    cdef cppclass SPI_Config:
        SPI_Config(Sequencer * sequencer, int address) except +
        void txrx(char * data, int len, int read_offset)

cdef class PySpi_Config:
    cdef SPI_Config *thisptr
    def __cinit__(self, PySequencer sequencer, int address):
        self.thisptr = new SPI_Config(sequencer.thisptr, address)
    def __dealloc__(self):
        del self.thisptr
    def txrx(self, wdata, int rcount=0):
        tdata = wdata + rcount*"\x00"
        self.thisptr.txrx(tdata, len(wdata) + rcount, len(wdata))
        return tdata[:rcount]

cdef extern from "TimeIt.h":
    cdef cppclass TimeIt:
        TimeIt() except +
        double elapsed()

cdef class PyTimeIt:
    cdef TimeIt *thisptr
    def __cinit__(self):
        self.thisptr = new TimeIt()
    def __dealloc__(self):
        del self.thisptr
    def elapsed(self):
        return self.thisptr.elapsed()

cdef extern from "Xilinx_DRP.h":
    cdef cppclass Xilinx_DRP:
        Xilinx_DRP(Sequencer * sequencer, int address) except +
        int read(int addr)
        void write(int addr, int data)

cdef class PyXilinx_DRP:
    cdef Xilinx_DRP *thisptr
    def __cinit__(self, PySequencer sequencer, int address):
        self.thisptr = new Xilinx_DRP(sequencer.thisptr, address)
    def __dealloc__(self):
        del self.thisptr
    def read(self, int address):
        return self.thisptr.read(address)
    def read(self, int address, int data):
        self.thisptr.write(address, data)
