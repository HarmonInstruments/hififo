/*
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

#include <unistd.h>
#include <stdlib.h>
#include <thread>
#include <iostream>
#include <stdexcept>
#include "Lvds_io.h"

using namespace std;

#define NTAPS 32 // number of taps in IODELAY
#define CALVALRX 0x080FF010 // calibration value output by target
#define CALVALTX 0x08555510 // calibration value looped back
#define W_RXTAP (1<<17)
#define W_TXTAP (1<<16)
#define WCYCLES 100 // number of cycles to wait for a write
#define RCYCLES 200 // number of cycles to wait for a read

/* return the most distant index from mismatched data */
// fail if all or none match
static int find_most_distant(bool * match)
{
	int distance;
	int vmax = 0;
	int imax = 0;
	int sum = 0;
	for(int i = 0; i<NTAPS; i++){
		if(match[i])
			sum++;
		// find closest mismatch
		for(int j=0; j<NTAPS; j++){
			distance = j;
			if(!match[(i+j)&(NTAPS-1)])
				break;
			if(!match[(i-j)&(NTAPS-1)])
				break;
		}
		if(distance > vmax){
			vmax = distance;
			imax = i;
		}
	}
	cout << "tap distance = " << vmax;
	if(sum == 0){ // none match, probably no link
		cout << " no matches\n";
		return -1;
	}
	if(sum == 32){ // all match, ???
		cout << " all match ???\n";
		return -2;
	}
	return imax;
}

static int scan_rx(Sequencer *seq, int addr)
{
	bool match[NTAPS];
	for(int i = 0; i<NTAPS; i++){
		seq->write(addr + 1, W_RXTAP | i, WCYCLES);
		seq->write(addr, 0, RCYCLES);
		uint32_t rval = seq->read(addr);
		//cout << "rx tap " << i << " read " << rval << endl;
		match[i] = rval == CALVALRX;
	}
	int tap = find_most_distant(match);
	if(tap < 0)
		return tap;
	seq->write(addr + 1, W_RXTAP | tap, WCYCLES);
	cout << ", set RX tap to " << tap << endl;
	return tap;
}

static int scan_tx(Sequencer *seq, int addr)
{
	bool match[NTAPS];
	for(int i = 0; i<NTAPS; i++){
		seq->write(addr + 1, W_TXTAP | i, WCYCLES);
		seq->write(addr, 0x000FFF00000000 | CALVALTX, RCYCLES);
		uint32_t rval = seq->read(addr);
		//cout << "tx tap " << i << " read " << rval << endl;
		match[i] = rval == CALVALTX;
	}
	int tap = find_most_distant(match);
	if(tap < 0)
		return tap;
	seq->write(addr + 1, W_TXTAP | tap, WCYCLES);
	cout << ", set TX tap to " << tap << endl;
	return tap;
}

Lvds_io::Lvds_io(Sequencer *sequencer, int addr)
{
	seq = sequencer;
	base = addr;
	cerr << "initializing LVDS IO, addr = " << base << "\n";
	scan_rx(seq, base);
	scan_tx(seq, base);
}

void Lvds_io::write(int addr, uint64_t data)
{
	uint64_t wdata = (((uint64_t) addr) << 32) | data;
	seq->write(base, wdata, RCYCLES);
}

uint32_t Lvds_io::read(int addr)
{
	return seq->read(base);
}

void Lvds_io::reset(void)
{
	seq->write(6,1, WCYCLES + 1000);
	seq->write(6,0, WCYCLES);
	cout << "reset IO delay, ready = " << seq->read(6) << endl;
}



