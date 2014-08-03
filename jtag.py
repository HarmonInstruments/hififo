#!/usr/bin/env python

from xjtag import XilinxJtag

import array
import struct

if __name__ == "__main__":
    jtag = XilinxJtag(0x0403, 0x6014, 1)
    jtag.get_idcode_from_reset()
    jtag.get_idcode_from_instruction()
    jtag.get_dna()
    jtag.get_idcode_from_instruction()
    import sys
    if len(sys.argv) > 1:
        jtag.program(sys.argv[1])
    jtag.get_idcode_from_instruction()
    jtag.get_usercode()
