#!/usr/bin/env python

import array
import struct

class XADC():
    """Xilinx XADC"""
    def __init__(self, port):
        self.port = port

    def _write_xadc(self, reg, value):
        """Write an XADC register"""
        self.port.write(reg, value)

    def _read_xadc(self, reg):
        """Read an XADC register"""
        return self.port.read(reg)

    def print_xadc_temp(self, channel, name):
        for i in range(3):
            temp = self._read_xadc(channel[i]) * 503.975 / 65536.0 - 273.15
            print "{} = {:.3} degrees C".format(name[i], temp)

    def print_xadc_supply(self, channel, name, limit):
        name_prefix = ['', 'max ', 'min ']
        for i in range(3):
            voltage = self._read_xadc(channel[i]) * 3.0 / 65536.0
            print "{} = {:.3} V".format(name_prefix[i] + name, voltage)
            if voltage < limit[0] or voltage > limit[1]:
                print "WARNING: supply voltage out of range"

    def read_xadc(self):
        self.print_xadc_temp([0, 32, 36], ['temp', 'max_temp', 'min_temp'])
        self.print_xadc_supply([1, 33, 37], 'vccint', [0.97, 1.03])
        self.print_xadc_supply([2, 34, 38], 'vccaux', [1.71, 1.89])
        self.print_xadc_supply([6, 35, 39], 'vccbram', [0.97, 1.03])

        vin = self._read_xadc(3) * 1.0 / 65536.0
        print "vin = {:.4} V".format(vin)
