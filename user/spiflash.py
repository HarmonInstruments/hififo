#!/usr/bin/env python

import struct
import time

class SPIFlash():
    """SPI Flash"""

    def __init__(self, spidev):
        self.spidev = spidev
        self.sector_size = 65536
        self.page_size = 256
        self.power_up()

    def read_id(self):
        idcode = self.spidev.txrx("\x9F", 4)
        (hcode, ) = struct.unpack(">I", idcode)
        print "ID =", hex(hcode)

    def write_enable(self):
        self.spidev.txrx("\x06")

    def power_up(self):
        self.spidev.txrx("\xAB")
    def power_down(self):
        self.spidev.txrx("\xB9")

    def read(self, address, count):
        return self.spidev.txrx(struct.pack(">BI", 0x0B, address << 8), count)

    def read_status(self):
        return ord(self.spidev.txrx("\x05", 1)[0])

    def write_status(self, val):
        self.write_enable();
        self.spidev.txrx(struct.pack("BB", 0x01, val))
        self.wait_busy();

    def wait_busy(self):
        for i in range(10000):
            status = self.read_status()
            if (status & 0x01) == 0:
                return
            time.sleep(0.001)
        raise RuntimeError("SPI flash timeout")

    def page_program(self, address, data):
        if len(data) != 256:
            raise RuntimeError("page program length must be 256 bytes")
        if (address & 0xFF0000FF) != 0:
            raise RuntimeError(
                "page program address must be aligned to 256 bytes")
        self.write_enable()
        self.spidev.txrx(struct.pack(">I", 0x02000000 | address) + data)
        self.wait_busy()

    def sector_erase(self, address):
        self.write_enable()
        if (address & 0xFF00FFFF) != 0:
            raise RuntimeError(
                "sector erase address must be aligned to 65536 bytes")
        self.spidev.txrx(struct.pack(">I", 0xD8000000 | address))
        self.wait_busy()

    def write(self, address, data):
        if (address & (self.sector_size - 1)) != 0:
            raise RuntimeError("write address must be aligned to sector size")
        self.write_status(0)
        sectors = len(data) / self.sector_size
        if len(data) != sectors*self.sector_size:
            sectors += 1
            data += "\x00" * (sectors*self.sector_size - len(data))
        for i in range(sectors):
            print("erasing sector {} of flash".format(i))
            self.sector_erase(address + self.sector_size * i)
            print("writing sector {} of flash".format(i))
            for j in range(self.sector_size / self.page_size):
                startbyte = self.sector_size*i + self.page_size*j
                self.page_program(address + startbyte,
                                  data[startbyte:startbyte+256])
            print("reading back sector {} of flash".format(i))
            rb = self.read(address + self.sector_size*i, self.sector_size/2)
            rb += self.read(address + self.sector_size*i + self.sector_size/2, self.sector_size/2)
            if rb != data[i*self.sector_size:(i+1)*self.sector_size]:
                for i in range(32):
                    print hex(ord(rb[i])), hex(ord(data[i]))
                raise RuntimeError("SPI flash sector write failed")
        print("flash write complete")

    def write_file(self, address, filename):
        data = ""
        with open(filename, "rb") as f:
            data = f.read()
        self.write(address, data)

    def read_file(self, address, filename, length):
        data = self.read(address, length)
        with open(filename, "wb") as f:
            f.write(data)
