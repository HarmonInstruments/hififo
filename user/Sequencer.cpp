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

#include "Sequencer.h"

using namespace std;

Sequencer::Sequencer(const char * filename_write, const char * filename_read)
{
	wf = new Hififo {filename_write};
	rf = new Hififo {filename_read};
	bufsize = 384*1024;
	wptr = 0;
	wcount = 0;
	reads_expected = 0;
	reads_requested = 0;
	wbuf = NULL;
	rbuf = NULL;
}

void Sequencer::append(uint64_t data)
{
	if(wcount == bufsize)
		throw std::runtime_error(
			"sequencer request exceeds write buffer size in " \
			__FILE__ );
	if(wbuf == NULL)
		wbuf = (uint64_t *) wf->get_buffer(8*bufsize);
	wbuf[wcount++] = data;
}

void Sequencer::wait(uint64_t count)
{
	append(1L<<62 | 1L<<61 | count << 32 | 0);
}

void Sequencer::write_req(size_t count, uint32_t address, uint64_t * data)
{
	append(2L<<62 | 1L<<61 | count << 32 | address);
	while(count--)
		append(*data++);
}

void Sequencer::write_single(uint32_t address, uint64_t data)
{
	write_req(1, address, &data);
}

void Sequencer::read_req(size_t count, uint32_t address)
{
	if((reads_expected + count) > bufsize)
		throw std::runtime_error(
			"sequencer request exceeds read buffer size in " \
			__FILE__ );
	append(3L<<62 | 1L<<61 | count << 32 | address);
	reads_expected += count;
}

uint64_t * Sequencer::run()
{
	// generate a flush for the read FIFO
	size_t increment = 16; // 128 bytes
	size_t excess_reads = reads_expected % increment;
	if(excess_reads != 0)
		read_req(increment - excess_reads, 0);
	// generate a flush for the write FIFO
	increment = 64; // 512 bytes
	size_t excess_writes = wcount % increment;
	if(excess_writes != 0){
		size_t fill_count = increment - excess_writes;
		while(fill_count--)
			append(0);
	}
	if(reads_requested != 0)
		rf->put_buffer(8*reads_requested);
	reads_requested = 0;
	wf->put_buffer(8*wcount);
	wbuf = NULL;
	if(reads_expected != 0){
		rbuf = (uint64_t *) rf->get_buffer(8*reads_expected);
		if(!rbuf){
			throw std::runtime_error( "sequencer read timeout" );
		}
		reads_requested = reads_expected;
	}
	//cerr << "sequencer run: bytes written = " << 8*wcount << " bytes read = " << 8*reads_expected << "\n";
	wptr = 0;
	wcount = 0;
	reads_expected = 0;
	return rbuf;
}
