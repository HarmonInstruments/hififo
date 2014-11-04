#!/usr/bin/env python
"""
HIFIFO: Harmon Instruments PCI Express to FIFO
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

import pyhififo
import spiflash
import xadc

seq = pyhififo.PySequencer("/dev/hififo_0_1", "/dev/hififo_0_5")
spi = pyhififo.PySpi_Config(seq, 4)
drp_xadc = pyhififo.PyXilinx_DRP(seq, 5)
drp_gt0 = pyhififo.PyXilinx_DRP(seq, 8)

adc = xadc.XADC(drp_xadc)
adc.read_xadc()
flash = spiflash.SPIFlash(spi)

flash.read_id()

#flash.write_file(0, "top.bin")
#flash.read_file(0, "toprb.bin", 1048576)

dc = 4
a = flash.read(0, dc)
for i in range(dc):
    print hex(ord(a[i]))

for i in range(16):
    print "drp_", i, drp_gt0.read(i)

print "done"
