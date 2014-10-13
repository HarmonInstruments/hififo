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

#include "AlignedMem.h"
#include "Sequencer.h"

using namespace std;

Sequencer::Sequencer(const char * filename_write, const char * filename_read)
{
  wf = new Hififo_Write {filename_write};
  rf = new Hififo_Read  {filename_read};
  bufsize = 131072;
  wptr = 0;
  wcount = 0;
  reads_expected = 0;
  amem = new AlignedMem{2*8*bufsize, 1};
  wbuf = (uint64_t *) amem->addr();
  rbuf = &wbuf[bufsize];
}

void Sequencer::append(uint64_t data)
{
  if(wcount == bufsize)
    throw std::runtime_error( "sequencer request exceeds write buffer size in " __FILE__ );
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
    throw std::runtime_error( "sequencer request exceeds read buffer size in " __FILE__ );
  append(3L<<62 | 1L<<61 | count << 32 | address);
  reads_expected += count;
}

uint64_t * Sequencer::run()
{
  // generate a flush for the read FIFO
  int increment = 64;
  size_t excess_reads = reads_expected % increment;
  if(excess_reads != 0)
    read_req(increment - excess_reads, 0);
  // generate a flush for the write FIFO
  size_t excess_writes = wcount % increment;
  if(excess_writes != 0){
    size_t fill_count = increment - excess_writes;
    while(fill_count--)
      append(0);
  }
  wf->request(wbuf, 8*wcount);
  if(reads_expected != 0){
    size_t rc = rf->request(rbuf, 8*reads_expected);
    if(rc != 8*reads_expected){
      throw std::runtime_error( "sequencer read timeout" );
    }
    rf->wait_all();
  }
  //cerr << "sequencer run: bytes written = " << 8*wcount << " bytes read = " << 8*reads_expected << "\n";
  wf->wait_all();
  wptr = 0;
  wcount = 0;
  reads_expected = 0;
  return rbuf;
}
