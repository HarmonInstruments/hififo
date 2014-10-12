#!/usr/bin/env python

import struct
import time

class SPIFlash():
    """SPI Flash"""

    def __init__(self, spidev):
        self.spidev = spidev
        self.sector_size = 65536
        self.page_size = 256

    def read_id(self):
        idcode = self.spidev.txrx("\x9F", 4)
        (hcode, ) = struct.unpack(">I", idcode)
        print "ID =", hex(hcode)

    def write_enable(self):
        self.spidev.txrx("\x06")

    def read(self, address, count):
        return self.spidev.txrx(struct.pack(">BI", 0x0B, address << 8), count)

    def read_status(self):
        status = self.spidev.txrx("\x05", 1)[0]
        print "status =", hex(ord(status))
        return status

    def wait_busy(self):
        print "b"
        for i in range(2000):
            status = self.read_status()
            if (ord(status[0]) & 0x01) == 0:
                return
            time.sleep(0.001)
        raise

    def page_program(self, address, data):
        if len(data) != 256:
            raise Error("page program length must be 256 bytes")
        if (address & 0xFF0000FF) != 0:
            raise Error("page program address must be aligned to 256 bytes")
        self.write_enable()
        self.spidev.txrx(struct.pack(">I", 0x02000000 | address) + data)
        self.wait_busy()

    def bulk_erase(self):
        self.write_enable()
        self.spidev.txrx("\C7")
        self.wait_busy()

    def sector_erase(self, address):
        self.write_enable()
        if (address & 0xFF00FFFF) != 0:
            raise Error("sector erase address must be aligned to 65536 bytes")
        self.spidev.txrx(struct.pack(">I", 0xD8000000 | address))
        self.wait_busy()

    def write(self, address, data):
        if (address & (self.sector_size - 1)) != 0:
            raise Error("write address must be aligned to sector size")
        pages = len(data) / self.page_size
        if len(data) != pages*self.page_size:
            pages += 1
            data += "\x00" * (pages*self.page_size - len(data))
        sectors = len(data) / self.sector_size
        if len(data) != sectors*self.sector_size:
            sectors += 1
        for i in range(sectors):
            print("erasing sector {} of flash".format(i))
            self.sector_erase(address + self.sector_size * i)
        print("erase complete")
        for i in range(pages):
            startbyte = self.page_size*i
            #print("programming page {} of flash".format(i))
            self.page_program(address + self.page_size * i, data[startbyte:startbyte+256])
            print ".",
        print("program complete")

    def write_file(self, address, filename):
        data = ""
        with open(filename, "rb") as f:
            data = f.read()
        self.write(address, data)
