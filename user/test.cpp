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
#include <fcntl.h>
#include <stdint.h>
//#include <sys/mman.h>
//#include <sys/ioctl.h>
#include <time.h>
#include <thread>
#include <iostream>
#include <stdexcept>

#include "TimeIt.h"
#include "AlignedMem.h"
#include "Hififo.h"
#include "Sequencer.h"

using namespace std;

class SPI_Config {
private:
  Sequencer *seq;
  int spi_address;
public:
  SPI_Config(Sequencer *sequencer, int addr) {
    seq = sequencer;
    spi_address = addr;
  }
  void txrx(uint8_t * data, size_t len, ssize_t read_offset){
    for(size_t i=0; i<len; i++){
      seq->write_single(spi_address, ((i+1) == len ? 0 : 0x100) | data[i]);
      seq->wait(128);
      if((read_offset >= 0) && (i >= (unsigned) read_offset))
	seq->read_req(1, spi_address);
    } 
    if(read_offset < 0)
      return;
    uint64_t * rdata64 = seq->run();
    for(size_t i=read_offset; i<len; i++)
      data[i-read_offset] = rdata64[i];
  }
};

void init_array(uint64_t *buf, size_t count, uint64_t * start_count){
  for(size_t j=0; j<count; j++)
    buf[j] = (*start_count)++;
}

void writer(Hififo_Write *f, size_t count, int use_hp)
{
  ssize_t bs = 4*1024*1024;
  uint64_t wcount = 0xDEADE00000000000;
  AlignedMem mem = {(size_t) bs*16, use_hp};
  uint64_t *wbuf = (uint64_t *) mem.addr();
  init_array(wbuf, bs, &wcount);
  TimeIt timer{};
  for(uint64_t i=0; i<count; i+=bs){
    uint64_t * buf = &wbuf[bs & i];
    if(i!=0)
      {
	f->wait_all_but_one();
	init_array(buf, bs, &wcount);
      }
    ssize_t rc = f->request(buf, bs*8);
    if(rc != bs*8){
      cerr << "timed out, " << rc << " bytes of " << bs*8 << " bytes\n";
      throw std::runtime_error( "hififo write timeout" );
    }
  }
  f->wait_all();
  auto runtime = timer.elapsed();
  auto speed = count * 8.0 * 1.0e-6 / runtime;
  std::cerr << "wrote " << count*8 << " bytes in " << runtime << " seconds, " << speed << " MB/s\n";
};

void checker(Hififo_Read *f, size_t count, int use_hp)
{
  uint64_t expected = 0;
  ssize_t bs = 1*1024*1024;
  int prints = 0;
  f->init_buffer(bs*8*2);
  TimeIt timer{};
  for(uint64_t i=0; i<count; i+=bs){
    uint64_t *buf;
    try{
      if(bs*8 != (ssize_t) f->get_read_buffer((void **) (&buf))){
	cerr << "timed out\n";
	break;
      }
    }
    catch(const std::runtime_error & e){
      cerr << "runtime error " << e.what() << "\n";
      return;
    }
    for(unsigned int j=0; j<bs; j++){
      if(expected != buf[j])
	{
	  if(prints > 32)
	    break;
	  if(!((i==0) && (j==0)))
	    std::cerr << "error at " << i << " " << j << " rval = 0x" << std::hex << buf[j] << " expected = 0x" << expected << "\n" << std::dec;
	  prints++;
	}
      expected = buf[j] + 1;
    }
    f->put_read_buffer();
  }
  auto runtime = timer.elapsed();
  auto speed = count * 8.0e-6 / runtime;
  std::cerr << "read " << count*8L << " bytes in " << runtime << " seconds, " << speed << " MB/s\n";
}

int main ( int argc, char **argv )
{
  uint64_t length = 1048576L*64;

  Hififo_Write f2{"/dev/hififo_0_2"};
  Hififo_Read f6{"/dev/hififo_0_6"};
  Hififo_Write f0{"/dev/hififo_0_0"};
  Hififo_Read f4{"/dev/hififo_0_4"};

  Sequencer seq{"/dev/hififo_0_1", "/dev/hififo_0_5"};
  
  uint64_t * sbuf;

  TimeIt timer{};
  
  SPI_Config spi(&seq, 4);
  uint8_t spibuf_in[256];
  spibuf_in[0] = 0x9F;
  spibuf_in[1] = 0;
  spi.txrx(spibuf_in, 32, 0);
  
  cerr << "sequencer completed in " << timer.elapsed() << " seconds\n";

  for(int i=0; i<32; i++)
    cerr << "spibuf[" << i << "] = " << std::hex << 1L*spibuf_in[i] << "\n";

  for(int i=0; i<64; i++){
    seq.write_single(5, i<<16);
    seq.wait(100);
    seq.read_req(1, 5);
  }
  sbuf = seq.run();
  for(int i=0; i<32; i++)
    cerr << "xadc[" << i << "] = " << std::hex << sbuf[i] << "\n";

  for(int i=0; i<32; i++){
    seq.write_single(8, i<<16);
    seq.write_single(9, i<<16);
    seq.write_single(10, i<<16);
    seq.write_single(11, i<<16);
    seq.wait(10000);
    seq.read_req(4, 8);
  }
  sbuf = seq.run();
  for(int i=0; i<32; i++)
    cerr << "gt3[" << i << "] = " << std::hex << sbuf[i] << "\n";
  /*
  cerr << "one way, huge pages\n";
  writer(&f2, length, 1);
  checker(&f6, length, 1);
  
  std::cerr << "f0 -> f4\n";
  #pragma omp parallel sections
  {
    #pragma omp section
    {
      checker(&f4, length, 1);
    }
    #pragma omp section
    {
      writer(&f0, length, 1);
    }
  }
  
  std::cerr << "f2 -> f6\n";
  std::thread t_reader (checker, &f6, length, 1);
  std::thread t_writer (writer, &f2, length, 1);
  t_writer.join();
  t_reader.join();
  
  cerr << "running timeout tests\n";
  writer(&f0, 1024, 1);
  checker(&f4, length, 1);
  */
  return 0;
}
