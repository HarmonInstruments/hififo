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
#include <iostream>
#include <stdexcept>
#include <vector>

#include "Sequencer.h"

using namespace std;

Sequencer::Sequencer(const char * filename_write, const char * filename_read)
{
	wf = new Hififo {filename_write};
	rf = new Hififo {filename_read};
	reads_expected = 0;
}

Sequencer::~Sequencer()
{
	delete wf;
	delete rf;
}

void Sequencer::append(uint64_t data)
{
	wbufv.push_back(data);
}

void Sequencer::wait(uint64_t count)
{
	wbufv.push_back(1L<<62 | 1L<<61 | count << 32 | 0);
}

void Sequencer::write_req(size_t count, uint32_t address, uint64_t * data)
{
	wbufv.push_back(2L<<62 | 1L<<61 | count << 32 | address);
	while(count--)
		wbufv.push_back(*data++);
}

void Sequencer::write_single(uint32_t address, uint64_t data)
{
	write_req(1, address, &data);
}

void Sequencer::write(uint32_t address, uint64_t data, uint64_t count)
{
	write_single(address, data);
	wait(count);
	run();
}

uint64_t Sequencer::read(uint32_t address)
{
	uint64_t rv;
	read_multi(address, &rv, 1);
	return rv;
}

void Sequencer::read_multi(uint32_t address, void *data, uint64_t count)
{
	std::vector<uint64_t> rv = read_multi(address, count);
	memcpy(data, &rv[0], count*8);
}

std::vector<uint64_t> Sequencer::read_multi(uint32_t address, uint64_t count)
{
	if(count != 0)
		wbufv.push_back(3L<<62 | 0L<<61 | count << 32 | address);
	reads_expected += count;
	run();
	std::vector<uint64_t> rv(reads_expected);
	if(reads_expected != 0)
		rf->bread(&rv[0], 8*reads_expected);
	reads_expected = 0;
	return rv;
}

void Sequencer::read_req(size_t count, uint32_t address)
{
	append(3L<<62 | 1L<<61 | count << 32 | address);
	reads_expected += count;
}

void Sequencer::run()
{
	// generate a flush for the read FIFO
	size_t excess_reads = reads_expected % 16; // 128 bytes
	if(excess_reads != 0)
		read_req(16 - excess_reads, 0);
	// generate a flush for the write FIFO
	size_t excess_writes = wbufv.size() % 64; // 512 bytes
	if(excess_writes != 0){
		size_t fill_count = 64 - excess_writes;
		while(fill_count--)
			wbufv.push_back(0);
	}
	wf->bwrite((const char *) &wbufv[0], 8*wbufv.size());
	wbufv.clear();
}
