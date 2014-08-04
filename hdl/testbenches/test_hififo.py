#!/usr/bin/env python

# HIFIFO: Harmon Instruments PCI Express to FIFO
# Copyright (C) 2014 Harmon Instruments, LLC
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
# 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/
 
import sys, os, random, Queue
import numpy as np
from myhdl import *
 
def endianswap(x):
    x = int(x)
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
        self.completion_data = np.zeros(16*1048576, dtype = 'uint64')
        self.write_data = np.zeros(16*1048576, dtype = 'uint64')
        self.write_data_expected = np.zeros(16*1048576, dtype = 'uint64')
        self.complete_addr = 0
    def write(self, data, address):
        self.txqueue.put([0xbeef00ff40000002, 0])
        self.txqueue.put([address | (endianswap(data) << 32), 0])
        self.txqueue.put([endianswap(data >> 32), 1])
    def read(self, address, tag=0):
        self.txqueue.put([0xbeef00ff00000001 | tag << 40, 0])
        self.txqueue.put([address, 1])
        self.read_outstanding = 1
    def complete(self, address, reqid_tag):
        complete_size = random.choice([8,16]) # QW
        for i in range(64/complete_size):
            self.txqueue.put([0xbeef00004A000000 | complete_size | ((512-complete_size*8*i) << 32), 0]) # 8 DW
            self.txqueue.put([combine2dw((reqid_tag << 8) | ((i & 1) << 6), endianswap(self.completion_data[self.complete_addr])), 0])
            for j in range(complete_size):
                self.complete_addr += 1
                self.txqueue.put([combine2dw(endianswap(int(self.completion_data[self.complete_addr-1])>>32), endianswap(self.completion_data[self.complete_addr])), j==(complete_size - 1)])
        
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
        if random.choice([0,1,1,1]):
            try:
                a = self.txqueue.get(False)
                r_tvalid.next = 1
                r_tdata.next = a[0]
                r_tlast.next = a[1]
            except:
                r_tvalid.next = 0
        else:
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
                    print "{} bit write request at 0x{:016X}, {} dw, {} qw".format(bits, address, length, len(self.rxdata))
                    if length & 0x01 != 0:
                        print "ERROR: write request is an odd length"
                        return
                    if length/2 + 2 != len(self.rxdata):
                        print "ERROR: write request length does not match ", len(self.rxdata)
                    for i in range(length/2):
                        if bits == 32:
                            d = combine2dw(endianswap(self.rxdata[1+i] >> 32), endianswap(self.rxdata[2+i]))
                        else:
                            d = combine2dw(endianswap(self.rxdata[2+i]), endianswap(self.rxdata[2+i] >> 32))
                        self.write_data[address/8 + i & 0xFFFFFF] = d
                        print "d[{}] = 0x{:016X}".format(i, d)
                elif tlptype == 0b1001010: # read completion
                    data = endianswap(self.rxdata[1]>>32)
                    print "read completion, data = ", data
                    self.read_outstanding = 0
                else:
                    print "unknown TLP ", hex(self.rxdata[0])
                self.rxdata = []
        t_tready.next = random.choice([0,1,1,1])
        return

def seq_wait(x):
    return 1<<60 | x
def seq_put(x):
    return 2<<60 | x

p = PCIe_host()
# enable interrupts
p.write(0xF, 0*8)
# load page tables
for i in range(32):
    p.write(1024*1024*2*i | 0x1000000000, (32+i)*8)
for i in range(128):
    p.write(1024*1024*2*i | 0x1000000000, (128+i)*8)
# enable FIFOs
p.write(0, 8*8)
p.write(0x00040400, 3*8)
p.write(0x100, 4*8)
p.write(0x00300000, 6*8)
p.write(0x100, 7*8)
p.read(0*8, 1)
p.read(1*8, 1)
p.read(5*8, 1)
p.completion_data[0] = seq_wait(64)
p.completion_data[1] = seq_wait(64)
p.completion_data[3] = seq_wait(1)
p.completion_data[5] = seq_wait(0)
p.completion_data[8] = seq_put(64*32)
for i in range(64*32):
    p.completion_data[9+i] = i | 0xDEADBEEF00000000
    p.write_data_expected[i] = i | 0xDEADBEEF00000000
    
def hififo_v(clock, reset, t_tready, r_tvalid, r_tlast, r_tdata, interrupt, t_tdata, t_1dw, t_tlast, t_tvalid):
    r = os.system ("iverilog -DSIM -DTPC_CH=1 -DFPC_CH=1 -o tb_hififo.vvp tb_hififo.v ../hififo.v ../hififo_tpc_fifo.v ../hififo_fpc_fifo.v ../sync.v ../pcie_rx.v ../pcie_tx.v ../sequencer.v ../fifo.v ../block_ram.v ../hififo_fpc_mux.v")
    if r!=0:
        print "iverilog returned ", r
        exit(1)
    return Cosimulation("vvp -v -m /home/dlharmon/software/myhdl.vpi tb_hififo.vvp", **locals())
 
def _test_hififo():
    # to core
    clock = Signal(bool(0))
    reset = Signal(bool(1))
    t_tready = Signal(bool(0))
    r_tvalid = Signal(bool(0))
    r_tlast = Signal(bool(0))
    r_tdata = Signal(intbv(0, min=0, max=2**64-1))
    # from core
    interrupt = Signal(bool(0))
    t_tdata = Signal(intbv(0, min=0, max=2**64-1))
    t_1dw = Signal(intbv(0, min=0, max=2**8-1))
    t_tlast = Signal(bool(0))
    t_tvalid = Signal(bool(0))
    
    dut = hififo_v(clock, reset, t_tready, r_tvalid, r_tlast, r_tdata, interrupt, t_tdata, t_1dw, t_tlast, t_tvalid)
    
    @always(delay(1))
    def clockgen():
         clock.next = not clock

    @always(clock.posedge)
    def stim():
        p.docycle(reset, t_tdata, t_1dw, t_tlast, t_tvalid, t_tready, r_tvalid, r_tlast, r_tdata)
        if interrupt:
            print "interrupt"
  
    @instance
    def check():
        for i in range(10):
            yield clock.negedge
        reset.next = 0
        for i in range(0,12000,1):
            yield clock.posedge
        raise StopSimulation
    
    return dut, check, stim, clockgen
 
def test_hififo(n=None):
    Simulation(_test_hififo()).run(n)
    if (p.write_data_expected == p.write_data).all():
        print "PASS"
    else:
        print "FAIL"
        for i in range(len(p.write_data_expected)):
            a = p.write_data[i]
            b = p.write_data_expected[i]
            if a != b:
                print hex(int(a)),hex(int(b)),i

if __name__=="__main__":
    test_hififo()


