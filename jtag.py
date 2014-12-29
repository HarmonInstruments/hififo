#!/usr/bin/env python

from xjtag import XilinxJtag

import array
import struct

if __name__ == "__main__":
    import sys
    jtag = XilinxJtag(0x0403, 0x6014, 1)
    jtag.get_idcode_from_reset()
    #jtag.get_idcode_from_instruction()
    #jtag.get_dna()
    if len(sys.argv) > 1:
        if sys.argv[1] == 'xadc':
            jtag.read_xadc()
    #jtag.get_idcode_from_instruction()
    if len(sys.argv) > 1:
        jtag.program(sys.argv[1])
    jtag.get_idcode_from_instruction()
    jtag.get_usercode()
