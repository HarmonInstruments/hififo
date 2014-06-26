#!/usr/bin/env python
 
import sys, os, random, Queue
from myhdl import *
 
def hififo_v(clock, reset, t_tready, r_tvalid, r_tlast, r_tdata, tpc0_data, tpc0_write, fpc0_read, interrupt, t_tdata, t_1dw, t_tlast, t_tvalid, tpc0_ready, fpc0_data, fpc0_empty):
    r = os.system ("iverilog -DTPC_CH=1 -DFPC_CH=1 -o tb_hififo.vvp tb_hififo.v ../hififo.v ../hififo_tpc_fifo.v ../hififo_fpc_fifo.v ../sync.v ../sync_gray.v ../pcie_rx.v ../pcie_tx.v")
    if(r!=0):
        print "iverilog returned ", r
        exit(1)
    cmd = "vvp -v -m /home/dlharmon/software/myhdl.vpi tb_hififo.vvp"
    return Cosimulation(cmd,**locals())
 
def endianswap(x):
    x &= 0xFFFFFFFF
    y = (x&0xFF) << 24
    y |= (x&0xFF00) << 8
    y |= (x&0xFF0000) >> 8
    y |= (x&0xFF000000) >> 24
    return y

def combine2dw(l,h):
    return (l & 0xFFFFFFFF) | ((h & 0xFFFFFFFF) << 32)

class PCIe_host():
    def __init__(self):
        self.read_outstanding = 0
        self.rxdata = []
        self.requested = 0
        self.txqueue = Queue.Queue()
        self.complete_val = 0
        # enable interrupts
        self.write(0xF, 0*8)
        # load page tables
        self.write(0x100000000, 32*8)
        self.write(0x2000400000, 33*8)
        self.write(0x000000000, 64*8)
        self.write(0x100000000, 65*8)
        # enable FIFOs
        self.write(0, 8*8)
        self.write(0x00000100, 3*8)
        self.write(0x00300000, 6*8)
        self.read(5*8, 1)
    def write(self, data, address):
        self.txqueue.put([0xbeef00ff40000002, 0])
        self.txqueue.put([address | (endianswap(data) << 32), 0])
        self.txqueue.put([endianswap(data >> 32), 1])
        print "self.txqueue.qsize() = {}".format(self.txqueue.qsize())
    def read(self, address, tag=0):
        self.txqueue.put([0xbeef00ff00000002 | tag << 40, 0])
        self.txqueue.put([address, 1])
        print "self.txqueue.qsize() = {}".format(self.txqueue.qsize())
        self.read_outstanding = 1
    def complete(self, address, reqid_tag):
        complete_size = random.choice([8,16]) # QW
        for i in range(64/complete_size):
            self.txqueue.put([0xbeef00004A000000 | complete_size | ((512-complete_size*8*i) << 32), 0]) # 8 DW
            self.txqueue.put([combine2dw((reqid_tag << 8) | ((i & 1) << 6), endianswap(self.complete_val)), 0])
            for j in range(complete_size):
                self.complete_val += 1
                self.txqueue.put([combine2dw(0, endianswap(self.complete_val)), j==(complete_size - 1)])
                
        print "self.txqueue.qsize() = {}".format(self.txqueue.qsize())

    def do_read(self, address):
        self.txqueue.put(["read", address])
    def request(self):
        self.requested += 64
    def docycle(self, reset, t_tdata, t_1dw, t_tlast, t_tvalid, t_tready, r_tvalid, r_tlast, r_tdata):
        # handle tx
        if reset:
            r_tvalid.next = 0;
            t_tready.next = 0;
            return
        try:
            a = self.txqueue.get(False)
            r_tvalid.next = 1
            r_tdata.next = a[0]
            r_tlast.next = a[1]
        except:
            r_tvalid.next = 0
        # handle rx        
        if(t_tvalid & t_tready):
            self.rxdata.append(int(t_tdata))
            if t_1dw & ~t_tlast:
                print "Error: 1dw and not tlast"
            if(t_tlast):
                tlptype = (self.rxdata[0] >> 24) & 0x5F
                length = self.rxdata[0] & 0x3FF
                bits = 32
                if (self.rxdata[0] & 0x20000000) != 0:
                    bits = 64
                if bits == 32:
                    address = combine2dw(self.rxdata[1], 0)
                else:
                    address = combine2dw(self.rxdata[1] >> 32, self.rxdata[1])
                if tlptype == 0b0000000: # read
                    print "{} bit read request".format(bits),
                    print "at 0x{:016X}, tag = 0x{:02X}, length = {} dw".format(address, int(self.rxdata[0] >> 40 & 0xFF), length)
                    reqid_tag = 0xFFFFFF & (self.rxdata[0] >> 40) 
                    self.complete(address, reqid_tag)
                elif tlptype == 0b1000000: # write
                    print "{} bit write request at 0x{:016X}, {} dw".format(bits, address, length)
                    for i in range(length/2):
                        d = combine2dw(endianswap(self.rxdata[2+i]), endianswap(self.rxdata[2+i] >> 32))
                        if d != (((address/8) & 0xFFFF) + i): 
                            print "d[{}] = 0x{:016X}".format(i, d)
                elif tlptype == 0b1001010:
                    print "read completion,",
                    data = endianswap(self.rxdata[1]>>32)
                    data |= endianswap(self.rxdata[2])
                    print "data = ", data
                    self.read_outstanding = 0
                else:
                    print "unknown"
                    print "last", hex(self.rxdata[0])
                self.rxdata = []
        t_tready.next = random.choice([0,1])
        return
    def reset(self):
        pass

class Reader():
    def __init__(self):
        self.data = 0
        self.fail = False
        self.maxprints = 16
    def docycle(self, data, valid):
        if valid:
            if data != self.data:
                if self.maxprints != 0:
                    print "data mismatch, got {}, expected{}".format(data, self.data)
                    self.maxprints -= 1
                self.fail = True
            self.data += 1
        return random.choice([0,0,0,1])
    def reset(self):
        self.__init__()

class Wfifo():
    def __init__(self):
        self.count = 0
    def docycle(self, reset, ready):
        self.count += 1
        if reset:
            self.count = 0
            return (0,0)
        if ready and self.count > 7:
            return (self.count - 8, 1)
        else:
            return (0, 0)
    def reset(self):
        self.__init__()

class Rfifo():
    def __init__(self):
        self.expected = 0
        self.count = 0
        self.error = 0
    def docycle(self, data, valid):
        data = int(data)
        if valid:
            if data != self.expected:
                print "fpc0_data = {}, last+1 = {}".format(data, self.expected)
                self.error = 1
            self.expected = int(data) + 1
            self.count += 1
        return random.choice([0,1])
    
p = PCIe_host()

wf = Wfifo()
rf = Rfifo()
lastrx = 0

def _test_hififo():
    # to core
    clock = Signal(bool(0))
    reset = Signal(bool(1))
    t_tready = Signal(bool(0))
    r_tvalid = Signal(bool(0))
    r_tlast = Signal(bool(0))
    r_tdata = Signal(intbv(0, min=0, max=2**64-1))
    tpc0_data = Signal(intbv(0, min=0, max=2**64-1))
    tpc0_write = Signal(bool(0))
    fpc0_read = Signal(bool(0))
    # from core
    interrupt = Signal(bool(0))
    t_tdata = Signal(intbv(0, min=0, max=2**64-1))
    t_1dw = Signal(intbv(0, min=0, max=2**8-1))
    t_tlast = Signal(bool(0))
    t_tvalid = Signal(bool(0))
    tpc0_ready = Signal(bool(0))
    fpc0_data = Signal(intbv(0, min=0, max=2**64-1))
    fpc0_empty = Signal(bool(0))
    
    dut = hififo_v(clock, reset, t_tready, r_tvalid, r_tlast, r_tdata, tpc0_data, tpc0_write, fpc0_read, interrupt, t_tdata, t_1dw, t_tlast, t_tvalid, tpc0_ready, fpc0_data, fpc0_empty)
    
    @always(delay(1))
    def clockgen():
         clock.next = not clock

    @always(clock.posedge)
    def stim():
        p.docycle(reset, t_tdata, t_1dw, t_tlast, t_tvalid, t_tready, r_tvalid, r_tlast, r_tdata)
        (tpc0_data.next, tpc0_write.next) = wf.docycle(reset, tpc0_ready)
        if interrupt:
            print "interrupt"
        fpc0_read.next = rf.docycle(fpc0_data, fpc0_empty)

        if(reset):
            p.reset()
  
    @instance
    def check():
        for i in range(10):
            yield clock.negedge
        reset.next = 0
        for angle in range(0,1000000,1):
            yield clock.posedge
        raise StopSimulation
    
    return dut, check, stim, clockgen#instances()
 
def test_hififo(n=None):
    
    Simulation(_test_hififo()).run(n)
     
def trace_hififo(n=None):
    tb = traceSignals(_test_hififo)
    Simulation(tb).run(n)
 
if __name__=="__main__":
    test_hififo()
    print "fpc count, fpc error, tpc count", rf.count, rf.error, wf.count

