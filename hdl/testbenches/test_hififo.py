#!/usr/bin/env python
 
import sys, os, random, Queue
from myhdl import *
 
def hififo_v(clock, reset, t_tready, r_tvalid, r_tlast, r_tdata, tpc0_data, tpc0_write, fpc0_read, interrupt, t_tdata, t_1dw, t_tlast, t_tvalid, tpc0_ready, fpc0_data, fpc0_empty):
    os.system ("iverilog -o tb_hififo.vvp tb_hififo.v ../hififo.v ../hififo_tpc_fifo.v ../hififo_fpc_fifo.v ../sync.v ../sync_gray.v ../pcie_rx.v ../pcie_tx.v")
    cmd = "vvp -v -m /home/dlharmon/software/myhdl.vpi tb_hififo.vvp"
    return Cosimulation(cmd,**locals())
 
def endianswap(x):
    x &= 0xFFFFFFFF
    y = (x&0xFF) << 24
    y |= (x&0xFF00) << 8
    y |= (x&0xFF0000) >> 8
    y |= (x&0xFF000000) >> 24
    return y

class PCIe_host():
    def __init__(self):
        self.rxdata = []
        self.requested = 0
        self.txqueue = Queue.Queue()
    def write(self, data, address):
        self.txqueue.put([0xbeef00ff40000002, 0])
        self.txqueue.put([address | (endianswap(data) << 32), 0])
        self.txqueue.put([endianswap(data >> 32), 1])
        print "self.txqueue.qsize() = {}".format(self.txqueue.qsize())
    def read(self, address, tag=0):
        self.txqueue.put([0xbeef00ff00000002 | tag << 40, 0])
        self.txqueue.put([address, 1])
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
            print "writing TLP"
            r_tvalid.next = 1
            r_tdata.next = a[0]
            r_tlast.next = a[1]
        except:
            r_tvalid.next = 0
        # handle rx        
        t_tready.next = 1 # random.choice([0,1])
        if(t_tvalid & t_tready):
            self.rxdata.append(int(t_tdata))
            if t_1dw & ~t_tlast:
                print "Error: 1dw and not tlast"
            if(t_tlast):
                if self.rxdata[0] & 0xFFFFFFFFFFFF == 0x00084A000002:
                    print "read completion,",
                    data = endianswap(self.rxdata[1]>>32)
                    data |= endianswap(self.rxdata[2])
                    print "data = ", data
                elif self.rxdata[0] & 0xFF000000 == 0:
                    print "32 bit read request at 0x{:08X}, tag = 0x{:02X}".format(int(self.rxdata[1] & 0xFFFFFFFF), int(self.rxdata[0] >> 40 & 0xFF))
                else:
                    print "unknown"
                    print "last", hex(self.rxdata[0])
#2: tx_tdata <= {es(rc_data[31:0]), rc_rid_tag, 1'b0, rc_lower_addr, 3'd0}; // rc DW3, DW2
                    #3: tx_tdata <= {32'h0, es(rc_data[63:32])}; // rc DW4

                self.rxdata = []
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

p = PCIe_host()
p.write(0xDEADBEEF0BADC0DE, 0)
p.read(0)

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
        if(reset):
            p.reset()
            
    @instance
    def check():
        for i in range(6):
            yield clock.negedge
        reset.next = 0
        for angle in range(0,200,1):
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


