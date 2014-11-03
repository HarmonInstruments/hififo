/*
 * HIFIFO: Harmon Instruments PCI Express to FIFO
 * Copyright (C) 2014 Harmon Instruments, LLC
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/
 */

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <thread>
#include <iostream>
#include <stdexcept>

#include "Xilinx_DRP.h"

using namespace std;

Xilinx_DRP::Xilinx_DRP(Sequencer *sequencer, int addr)
{
	seq = sequencer;
	drp_address = addr;
	cerr << "opened Xilinx_DRP, addr = " << addr << "\n";
}

void Xilinx_DRP::write(int addr, int data)
{
	seq->write_single(drp_address, data | (addr << 16) | (1<<31));
	seq->wait(1000);
	seq->run();
}

int Xilinx_DRP::read(int addr)
{
	seq->write_single(drp_address, addr << 16);
	seq->wait(1000);
	seq->read_req(1, drp_address);
	uint64_t * rdata = seq->run();
	return rdata[0];
}
