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
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <time.h>
#include <thread>
#include <iostream>
#include <stdexcept>

using namespace std;

class TimeIt {
public:
  struct timespec start_time;
  TimeIt() {
    clock_gettime(CLOCK_REALTIME, &start_time);
  }
  double elapsed(void){
    struct timespec stop_time;
    clock_gettime(CLOCK_REALTIME, &stop_time);
    return (stop_time.tv_sec - start_time.tv_sec) + 1e-9 * (stop_time.tv_nsec - start_time.tv_nsec);  
  };
};

#define IOC_INFO 0x10

#define min(x,y) ((x) > (y) ? (y) : (x))
#define max(x,y) ((x) < (y) ? (y) : (x))

class AlignedMem {
private:
  void * buf;
  size_t len;
public:
  AlignedMem(size_t size, int use_hp) {
    len = size;
    cerr << "allocating\n";
    if(use_hp){
      buf = (uint64_t *) mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_POPULATE | MAP_HUGETLB, 1, 0); 
      if(buf != MAP_FAILED){
	return;
      }
      cerr << "falling back to regular pages\n";
    }
    buf = (uint64_t *) mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_POPULATE, 1, 0); 
    if (buf != MAP_FAILED)
      return;
    throw std::bad_alloc();
  }
  AlignedMem(size_t size) {AlignedMem{size, 1};}
  ~AlignedMem(){
    cerr << "deallocating\n";
    munmap(buf, len);
  }
  void *addr(){
    return buf;
  }
};

struct hififo_ioctl{
  void *buf;
  ssize_t length;
  int32_t wait;
  int32_t abort;
  int32_t timeout;
  int32_t info;
};

class Hififo {
private:
  int fd;
  int32_t finfo = 0;
  AlignedMem *amem;
protected:
  void *buf[2];
  ssize_t bufsize[2];
  ssize_t half_size; 
  int p_in;
  int p_out;
  int p_req;
  ssize_t ioc(void * buf, ssize_t count, int wait, int abort){
    struct hififo_ioctl tmp;
    tmp.length = count;
    tmp.buf = buf;
    tmp.wait = wait;
    tmp.abort = abort;
    if(ioctl(fd, _IOWR('f', IOC_INFO, struct hififo_ioctl), &tmp) != 0){
      throw std::runtime_error( "hififo device info ioctl failed" );
    }
    finfo = tmp.info;
    return tmp.length;
  }
public:
  Hififo(const char * filename) {
    fd = open(filename, O_RDWR);
    if(fd < 0){
      perror(filename);
      cerr << "fifo_open(" << filename << ") failed\n";
      throw std::runtime_error( "hififo open failed" );
    }
  }
  ~Hififo() {
    cerr << "closing hififo\n";
    close(fd);
  }
  int bwrite(const char *buf, size_t count){
    ssize_t rc = write(fd, buf, count);
    if((size_t) rc != count){
      std::cerr << "failed to write, attempted " << count << " bytes, retval = " << rc << "\n";
      throw std::runtime_error( "hififo write failed" );
    }
    return rc;
  }
  ssize_t bread(char * buf, size_t count){
    ssize_t rc = read(fd, buf, count);
    if((size_t) rc != count){
      std::cerr << "failed to read, attempted " << count << " bytes, retval = " << rc << "\n";
      throw std::runtime_error( "hififo read failed" );
    }
    return rc;
  }
  ssize_t request(void * buf, ssize_t count){
    return ioc(buf, count, 1, 0);
  }
  ssize_t wait_all(){
    return ioc(NULL, 0, 0, 0);
  }
  ssize_t wait_all_but_one(){
    return ioc(NULL, 0, -1, 0);
  }
  ssize_t abort(){
    return ioc(NULL, 0, 1, 1);
  }
  void init_buffer(size_t size){
    amem = new AlignedMem{size,1};
    half_size = size / 2;
    buf[0] = amem->addr();
    buf[1] = ((char *)buf[0]) + half_size;
    p_in = 0;
    p_out = 0;
    p_req = 0;
  }
};

class Hififo_Read : public Hififo 
{
private:
  void update_request(){
    while((p_req - p_out) < 2){
      //TimeIt timer{};
      ssize_t rc = request(buf[p_req & 1], half_size);
      if(rc != half_size){
	throw std::runtime_error( "hififo read timeout" );
      }
      p_req ++;
      //cerr << "req " << p_req << " " << p_out << " " << "update_request completed in " << timer.elapsed() << " seconds\n";
    }
  }
public:
  Hififo_Read(const char * filename)
  :Hififo(filename) {}

  size_t get_read_buffer(void ** buffer){
    //cerr << "grb\n";
    //TimeIt timer{};
    update_request();
    ssize_t bytes_remaining = wait_all_but_one();
    *buffer = buf[p_out & 1];
    //cerr << "grb completed in " << timer.elapsed() << " seconds, bytes_remaining = " << bytes_remaining << "\n";
    if(bytes_remaining < 0){
      cerr << "bytes_remaining = " << bytes_remaining << "\n";
      throw std::runtime_error( "hififo read timeout 1" );
    }
    return half_size - bytes_remaining;
  }
  void put_read_buffer(){
    p_out ++;
    update_request();
  }
};

class Hififo_Write : public Hififo 
{
private:
  void update_request(){
    while((p_req - p_out) < 2){
      TimeIt timer{};
      ssize_t rc = request(buf[p_req & 1], half_size);
      if(rc != half_size){
	cerr << "read timeout";
	break;
      }
      p_req ++;
      cerr << "req " << p_req << " " << p_out << " " << "update_request completed in " << timer.elapsed() << " seconds\n";
    }
  }
public:
  Hififo_Write(const char * filename)
  :Hififo(filename) {}
  size_t get_write_buffer(void ** buffer){
    cerr << "gwb\n";
    TimeIt timer{};
    update_request();
    ssize_t bytes_remaining = wait_all_but_one();
    *buffer = buf[p_out & 1];
    cerr << "gwb completed in " << timer.elapsed() << " seconds\n";
    if(bytes_remaining < 0)
      throw std::runtime_error( "hififo write timeout" );
    return half_size - bytes_remaining;
  }
  void put_write_buffer(){
    p_out ++;
    update_request();
  }
};

class Sequencer {
private:
  Hififo_Write * wf;
  Hififo_Read * rf;
  AlignedMem *amem;  
  uint64_t * wbuf;
  uint64_t * rbuf;
  size_t wcount;
  size_t wptr;
  size_t bufsize;
  size_t reads_expected;
public:
  Sequencer(const char * filename_write, const char * filename_read) {
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
  void append(uint64_t data){
    if(wcount == bufsize)
      throw;
    wbuf[wcount++] = data;
  }
  void wait(uint64_t count){
    append(1L<<62 | 1L<<61 | count << 32 | 0);
  }
  void write_req(size_t count, uint32_t address, uint64_t * data){
    append(2L<<62 | 1L<<61 | count << 32 | address);
    while(count--)
      append(*data++);
  }
  void write_single(uint32_t address, uint64_t data){
    write_req(1, address, &data);
  }
  void read_req(size_t count, uint32_t address){
    append(3L<<62 | 1L<<61 | count << 32 | address);
    reads_expected += count;
  }
  uint64_t * run(){
    // generate a flush for the read FIFO
    int increment = 1024;
    size_t excess_reads = reads_expected % increment;
    if(excess_reads != 0)
      read_req(increment - excess_reads, 0);
    // generate a flush for the write FIFO
    size_t excess_writes = wcount % increment;
    size_t fill_count = increment - excess_writes;
    while(fill_count--)
      append(0);
    wf->request(wbuf, 8*wcount);
    if(reads_expected != 0){
      size_t rc = rf->request(rbuf, 8*reads_expected);
      if(rc != 8*reads_expected){
	throw std::runtime_error( "sequencer read timeout" );
      }
      rf->wait_all();
    }
    wf->wait_all();
    wptr = 0;
    wcount = 0;
    reads_expected = 0;
    return rbuf;
  }
};

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
  /*
  Hififo_Write f2{"/dev/hififo_0_2"};
  Hififo_Read f6{"/dev/hififo_0_6"};
  Hififo_Write f0{"/dev/hififo_0_0"};
  Hififo_Read f4{"/dev/hififo_0_4"};
  */
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
    }*/
  /*
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
