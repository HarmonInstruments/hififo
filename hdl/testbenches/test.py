#!/usr/bin/env python

"""HIFIFO: Harmon Instruments PCI Express to FIFO
Copyright (C) 2014 Harmon Instruments, LLC

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/

"""

import sys, os, random
import numpy as np
import cocotb
from cocotb.clock import Clock
from cocotb.triggers import Timer, RisingEdge, ReadOnly, Event
from cocotb.result import TestFailure, ReturnValue

"""@cocotb.coroutine
def do_rotate(dut, din, angle):
#This coroutine performs a write of the RAM
    yield RisingEdge(dut.clock)
    angle_scaled = int(round(amult * angle))
    dut.angle = angle_scaled
    in_scaled = np.round(din*dmult)
    dut.in_re = int(np.real(in_scaled))
    dut.in_im = int(np.imag(in_scaled))
    for i in range(stages):
        yield RisingEdge(dut.clock)
    yield ReadOnly() # Wait until all events have executed for this timestep
    result = int(dut.out_re.value.signed_integer) + 1j* int(dut.out_im.value.signed_integer)
    expected = gain * in_scaled * np.exp(1j*angle_scaled/amult)
    error = np.abs(result - expected) / np.abs(expected)
    if error > 5e-4:
        dut.log.error("FAIL: din = {}, angle = {}, result = {}, expected = {}, error = {}".format(din, angle, result, expected, error))
        raise TestFailure("incorrect result")
    raise ReturnValue(result)
"""

@cocotb.test()
def run_test(dut):
    """Test HIFIFO"""
    clock = dut.vna_dsp.hififo.pcie_core_wrap.clock
    a = cocotb.fork(Clock(clock, 2500).start())
    pci = PCIe_host()
    b = cocotb.fork(pci.docycle(dut.vna_dsp.hififo.pcie_core_wrap))
    # enable interrupts
    pci.write(0xF, 0*8)
    # release reset
    pci.write(0xFF, 4*8)
    #FPC
    for i in range(64*32):
        pci.completion_data[0x21000/8+i] = i | 0xDEADBEEF00000000
        pci.write_data_expected[0x21000/8+i] = i | 0xDEADBEEF00000000
    pci.completion_data[0] = 2 | 0x21000
    pci.completion_data[1] = 3 | 0x200
    pci.completion_data[2] = 2 | 0x21200
    pci.completion_data[3] = 3 | 0x200
    pci.completion_data[63] = 4 | 0x200
    pci.completion_data[64] = 2 | 0x21400 | 0x100000000
    pci.completion_data[65] = 3 | 0x3C00
    pci.write_fifo(fifo=0, address=0xDEAD00000000)
    # TPC
    for i in range(0x8000/8):
        pci.write_data_expected[0x10000/8+i] = i
    pci.completion_data[256] = 2 | 0x300010000
    pci.completion_data[257] = 3 | 0x2000
    pci.completion_data[258] = 2 | 0x12000
    pci.completion_data[259] = 3 | 0x2000
    pci.completion_data[260] = 2 | 0x14000
    pci.completion_data[261] = 3 | 0x4000
    pci.write_fifo(fifo=6, address=0x800)
    for i in range(30):
        pci.completion_data[512+2*i] = 2 | 0x21000+0x200*i
        pci.completion_data[513+2*i] = 3 | 0x200
    pci.completion_data[512+63] = 4 | 0x1200;
    for i in range(30):
        pci.completion_data[512+64+2*i] = 2 | 0x21000+0x200*(i+30)
        pci.completion_data[513+64+2*i] = 3 | 0x200
    pci.completion_data[512+64+63] = 4 | 0x1400;
    for i in range(30):
        pci.completion_data[512+128+2*i] = 2 | 0x21000+0x200*(i+60)
        pci.completion_data[513+128+2*i] = 3 | 0x200
        pci.completion_data[512+128+63] = 4 | 0x1600;
    for i in range(6):
        pci.completion_data[512+192+2*i] = 2 | 0x21000+0x200*(i+90)
        pci.completion_data[513+192+2*i] = 3 | 0x200
    for i in range(32):
        print i, hex(int(pci.completion_data[512+i]))
    pci.write_fifo(fifo=4, address=0x1000)
    a = yield pci.read(0*8, 2)
    print a
    a = yield pci.read(1*8, 3)
    print a
    a = yield pci.read(2*8, 4)
    print a
    for i in range(17000):
        yield RisingEdge(clock)    
    if not (pci.write_data_expected == pci.write_data).all():
        print "FAIL - data, expected"
        fails = 0
        for i in range(len(pci.write_data_expected)):
            a = pci.write_data[i]
            b = pci.write_data_expected[i]
            if a != b:
                fails += 1
                print hex(int(a)),hex(int(b)),i
                if fails == 32:
                    break
        raise TestFailure("FAIL - data doesn't match")

import Queue

def endianswap(x):
    x = int(x)
    y = (x&0xFF) << 24
    y |= (x&0xFF00) << 8
    y |= (x&0xFF0000) >> 8
    y |= (x&0xFF000000) >> 24
    return y

class PCIe_host():
    def __init__(self):
        self.read_outstanding = 0
        self.rxdata = []
        self.requested = 0
        self.txqueue = Queue.Queue()
        self.completion_data = np.zeros(1048576, dtype = 'uint64')
        self.write_data = np.zeros(16*1048576, dtype = 'uint64')
        self.write_data_expected = np.zeros(16*1048576, dtype = 'uint64')
        self.errors = 0
        self.read_tag = 0
        self.read_data = 0
    def write(self, data, address):
        """32 bit address / 64 bit data write"""
        tlp = np.zeros(6, dtype='uint32')
        tlp[0] = 0x40000002
        tlp[1] = 0xbeef00ff
        tlp[2] = address
        tlp[3] = endianswap(data)
        tlp[4] = endianswap(data >> 32)
        self.txqueue.put(tlp)
    
    @cocotb.coroutine
    def read(self, address, tag=0):
        """ 32 bit read """
        tlp = np.zeros(4, dtype='uint32')
        tlp[0] = 0x00000001
        tlp[1] = 0xbaaa00ff | tag << 8
        tlp[2] = address
        self.read_wait = Event("rw")
        self.txqueue.put(tlp)
        yield self.read_wait.wait()
        raise ReturnValue(self.read_data)
    
    def complete(self, address, reqid_tag):
        address &= 0xFFFF8
        address = address >> 3
        complete_size = random.choice([16,32]) # DW
        cdata_64 = self.completion_data[address:address + 512/8]
        cdata_32 = np.fromstring(cdata_64.tostring(), dtype = 'uint32')
        cdata_32.byteswap(True)
        for i in range(128/complete_size):
            tlp = np.zeros(4 + complete_size, dtype='uint32')
            tlp[0] = 0x4A000000 | complete_size
            tlp[1] = 0xbeef0000 | (512-complete_size*4*i)
            tlp[2] = (reqid_tag << 8) | ((i & 1) << 6)
            tlp[3:3+complete_size] = cdata_32[i*complete_size: (i+1)*complete_size]
            address += complete_size/2
            self.txqueue.put(tlp)

    @cocotb.coroutine
    def do_tx(self, p):
        while True:
            try:
                tlp = self.txqueue.get(False)
                i_last = len(tlp)/2 - 1
                for i in range(len(tlp)/2):
                    while random.choice([0,0,0,1]):
                        p.m_axis_rx_tvalid = 0
                        yield RisingEdge(p.clock)
                    p.m_axis_rx_tvalid = 1
                    p.m_axis_rx_tdata = tlp[2*i] | (int(tlp[2*i+1]) << 32)
                    if i == i_last:
                        p.m_axis_rx_tlast = 1
                    else:
                        p.m_axis_rx_tlast = 0
                    yield RisingEdge(p.clock)
            except Queue.Empty:
                p.m_axis_rx_tvalid = 0
                yield RisingEdge(p.clock)

    @cocotb.coroutine
    def docycle(self, dut):
        p = dut
        p.m_axis_tx_tvalid = 0
        p.s_axis_tx_tready = 0
        p.pci_reset = 1
        for i in range(10):
            yield RisingEdge(p.clock)
        p.pci_reset = 0
        for i in range(10):
            yield RisingEdge(p.clock)
        cocotb.fork(self.do_tx(p))
        while True:
            # RX
            if (int(p.s_axis_tx_tvalid) == 1) and (int(p.s_axis_tx_tready == 1)):
                tdata = int(p.s_axis_tx_tdata)
                tlast = int(p.s_axis_tx_tlast)
                t_1dw = int(p.s_axis_tx_1dw)
                self.rxdata.append(tdata & 0xFFFFFFFF)
                self.rxdata.append(tdata >> 32)
                if (t_1dw == 1) and (tlast == 0):
                    raise TestFailure("RX: Error: 1dw and not tlast")
                if tlast == 1:
                    dw0 = self.rxdata[0]
                    dw1 = self.rxdata[1]
                    dw2 = self.rxdata[2]
                    dw3 = self.rxdata[3]
                    tlptype = (dw0 >> 24) & 0x5F
                    length = dw0 & 0x3FF
                    if (dw0 & 0x20000000) != 0:
                        bits = 64
                        address = dw3 | (int(dw2) << 32)
                    else:
                        bits = 32
                        address = dw2
                    if tlptype == 0b0000000: # read
                        #print "{} bit read request at 0x{:016X}, tag = 0x{:02X}, length = {} dw".format(bits, address, int(dw1 >> 8 & 0xFF), length)
                        reqid_tag = 0xFFFFFF & (dw1 >> 8)
                        self.complete(address, reqid_tag)
                    elif tlptype == 0b1000000: # write
                        self.handle_write_tlp(bits, address, length, self.rxdata)
                    elif tlptype == 0b1001010: # read completion
                        self.handle_read_completion_tlp(bits, length, self.rxdata)
                    else:
                        raise TestFailure("RX: unknown TLP {} {} {}".format(hex(dw0), hex(dw1), hex(dw2)))
                    self.rxdata = []
            p.s_axis_tx_tready = random.choice([0,1,1,1])
            yield RisingEdge(p.clock)
    def handle_read_completion_tlp(self, bits, length, rxdata):
        if length != 1:
            raise TestFailure("ERROR: read completion TLP has incorrect length in header, {}".format(length))
        if len(rxdata) != 4:
            raise TestFailure("ERROR: read completion TLP length does not match header, {}".format(len(self.rxdata)))
        data = endianswap(rxdata[3])
        tag = 0xFF & (rxdata[2]>>8)
        print "read completion, data = {}, tag = {}".format(data, tag)
        self.read_data = data
        self.read_wait.set(data)

    def handle_write_tlp(self, bits, address, length, rxdata):
        #print "{} bit write request at 0x{:016X}, {} header dw, {} payload dw".format(bits, address, length, len(rxdata))
        #if random.choice([0,0,0,0,1]):
            #self.read(0x1, self.read_tag)
            #self.read_tag += 1
        if length & 0x01 != 0:
            raise TestFailure("write request is an odd length")
        if length + 4 != len(self.rxdata):
            raise TestFailure("write request length does not match".format(len(rxdata)))
        ndata = np.asarray(rxdata, dtype = 'uint32')
        if bits == 32:
            data = ndata[3:3+length]
        else:
            data = ndata[4:4+length]
        data.byteswap(True)
        addr_start = (address/8) & 0xFFFFFF
        d = np.fromstring(data.tostring(), dtype = 'uint64')
        self.write_data[addr_start:addr_start + length/2] = d
        #print "d[{}] = 0x{:016X}".format(0, d[0])
    def write_fifo(self, fifo, address):
        self.write(address | 4, (8+fifo)*8)

def seq_wait(x):
    return 1<<60 | x
def seq_put(x):
    return 2<<60 | x


