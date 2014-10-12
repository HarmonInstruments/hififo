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

seq = pyhififo.PySequencer("/dev/hififo_0_1", "/dev/hififo_0_5")

"""for i in range(64):
    seq.write_single(5, i<<16);
    seq.wait(100);
    seq.read_req(1, 5);
sbuf = seq.run();
for i in range(32):
    print "xadc[{}] = {}".format(i, hex(sbuf[i]))
"""
spi = pyhififo.PySpi_Config(seq, 4)

flash = spiflash.SPIFlash(spi)

print flash.read_id()
t = pyhififo.PyTimeIt()
flash.sector_erase(0)
print "erased in ", t.elapsed(), " seconds"
t = pyhififo.PyTimeIt()
flash.sector_erase(65536)
print "erased in ", t.elapsed(), " seconds"

t = pyhififo.PyTimeIt()
flash.bulk_erase()
print "erased in ", t.elapsed(), " seconds"

#flash.page_program(0, "\x03"*256)
#a = flash.read(0, 1024)

#flash.write_file(0, "top.bin")

#print hex(ord(a[0]))
#print hex(ord(a[1]))
#print hex(ord(a[256]))
print "done"

"""
int main ( int argc, char **argv )
{
  uint64_t * sbuf;

  for(int i=0; i<64; i++){
    seq.write_single(5, i<<16);
    seq.wait(100);
    seq.read_req(1, 5);
  }
  sbuf = seq.run();
  for(int i=0; i<32; i++)
    cerr << "xadc[" << i << "] = " << std::hex << sbuf[i] << "\n";

  for(int i=0; i<32; i++){
    seq.write_single(8, i<<16);
    seq.write_single(9, i<<16);
    seq.write_single(10, i<<16);
    seq.write_single(11, i<<16);
    seq.wait(10000);
    seq.read_req(4, 8);
  }
  sbuf = seq.run();
  for(int i=0; i<32; i++)
    cerr << "gt3[" << i << "] = " << std::hex << sbuf[i] << "\n";
  return 0;
}
"""
