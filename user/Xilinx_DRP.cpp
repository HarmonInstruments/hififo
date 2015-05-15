/*
 * HIFIFO: Harmon Instruments PCI Express to FIFO
 * Copyright (C) 2014-2015 Harmon Instruments, LLC
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
#include <stdio.h>
#include <stdint.h>
#include <stdexcept>

#include "Xilinx_DRP.h"

using namespace std;

Xilinx_DRP::Xilinx_DRP(Sequencer *sequencer, int addr)
{
	seq = sequencer;
	drp_address = addr;
	fprintf(stderr, "opened Xilinx_DRP, addr = %d\n", addr);
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
	return seq->read(drp_address);
}

double Xilinx_DRP::xadc_temp(int channel)
{
	return read(channel) * 503.975 / 65536.0 - 273.15;
}

double Xilinx_DRP::xadc_supply(int channel)
{
	return read(channel) * 3.0 / 65536.0;
}

void Xilinx_DRP::xadc_print(){
	fprintf(stderr, "temp = %.2lf degrees C\n", xadc_temp(0));
	fprintf(stderr, "temp_max = %.2lf degrees C\n", xadc_temp(32));
	fprintf(stderr, "temp_min = %.2lf degrees C\n", xadc_temp(36));
	fprintf(stderr, "vccint = %.3lf V\n", xadc_supply(1));
	fprintf(stderr, "vccint_max = %.3lf V\n", xadc_supply(33));
	fprintf(stderr, "vccint_min = %.3lf V\n", xadc_supply(37));
	fprintf(stderr, "vccaux = %.3lf V\n", xadc_supply(2));
	fprintf(stderr, "vccaux_max = %.3lf V\n", xadc_supply(34));
	fprintf(stderr, "vccaux_min = %.3lf V\n", xadc_supply(38));
	fprintf(stderr, "vccbram = %.3lf V\n", xadc_supply(6));
	fprintf(stderr, "vccbram_max = %.3lf V\n", xadc_supply(35));
	fprintf(stderr, "vccbram_min = %.3lf V\n", xadc_supply(39));
	fprintf(stderr, "vin = %.3lf V\n", read(3) * 1.0 / 65536.0);
}
