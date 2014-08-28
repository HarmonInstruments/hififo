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
#define IOC_GET_TO_PC 0x11
#define IOC_PUT_TO_PC 0x12
#define IOC_GET_FROM_PC 0x13
#define IOC_PUT_FROM_PC 0x14
#define IOC_SET_TIMEOUT 0x15

#define min(x,y) ((x) > (y) ? (y) : (x))
#define max(x,y) ((x) < (y) ? (y) : (x))
/*
uint64_t fifo_set_timeout(struct fifodev *f, uint64_t timeout){
  if(ioctl(f->fd, _IO('f', IOC_SET_TIMEOUT), timeout) != 0){
    perror("hififo.c: fifo_set_timeout() failed");
    return -1;
  }
  return 0;
  }*/

class AlignedMem {
private:
  void * buf;
  size_t len;
public:
  AlignedMem(size_t size, int use_hp) {
    len = size;
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
  AlignedMem(size_t size) {
    AlignedMem{size, 1};
  }
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
  AlignedMem *ringbuf;
  size_t ringbuf_half_size;

  ssize_t ioc(void * buf, ssize_t count, int wait, int abort){
    struct hififo_ioctl tmp;
    tmp.length = count;
    tmp.buf = buf;
    tmp.wait = wait;
    tmp.abort = abort;
    if(ioctl(fd, _IOWR('f', IOC_INFO, struct hififo_ioctl), &tmp) != 0){
      perror("device info ioctl failed");
      throw;
    }
    finfo = tmp.info;
    //cerr << "tmp.length = " << tmp.length << "\n";
    return tmp.length;
  }
public:
  Hififo(const char * filename) {
    fd = open(filename, O_RDWR);
    if(fd < 0){
      perror(filename);
      cerr << "fifo_open(" << filename << ") failed\n";
      throw;
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
      throw 90;
    }
    return rc;
  }
  ssize_t bread(char * buf, size_t count){
    ssize_t rc = read(fd, buf, count);
    if((size_t) rc != count){
      std::cerr << "failed to read, attempted " << count << " bytes, retval = " << rc << "\n";
      //throw;
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
    ringbuf = new AlignedMem{size,1};
    ringbuf_half_size = size/2;
  }
  void * get_write_buffer(){
    return ringbuf;
  }
};

void init_array(uint64_t *buf, size_t count, uint64_t * start_count){
  for(size_t j=0; j<count; j++)
    buf[j] = (*start_count)++;
}

void writer(Hififo *f, size_t count, int use_hp)
{
  ssize_t bs = 1024*1024;
  ssize_t rc;
  uint64_t wcount = 0xDEADE00000000000;
  AlignedMem mem = {bs*16, use_hp};
  uint64_t *wbuf = (uint64_t *) mem.addr();
  init_array(&wbuf[0], bs, &wcount);
  TimeIt timer{};
  for(uint64_t i=0; i<count; i+=2*bs){
    rc = f->request(wbuf, bs*8);
    if(rc != bs*8){
      cerr << "timed out, " << rc << " bytes of " << bs*8 << " bytes\n";
      break;
    }
    f->wait_all_but_one();
    init_array(&wbuf[bs], bs, &wcount);
    rc = f->request(&wbuf[bs], bs*8);
    if(rc != bs*8){
      cerr << "timed out, " << rc << " bytes of " << bs*8 << " bytes\n";
      break;
    }
    f->wait_all_but_one();
    init_array(&wbuf[0] , bs, &wcount);
  }
  f->wait_all();
  auto runtime = timer.elapsed();
  auto speed = count * 8.0e-6 / runtime;
  std::cerr << "wrote " << count*8 << " bytes in " << runtime << " seconds, " << speed << " MB/s\n";
};

void checker(Hififo *f, size_t count, int use_hp)
{
  uint64_t expected = 0;
  size_t bs = 1024*512*8;
  int prints = 0;
  AlignedMem mem = {bs*8, use_hp};
  uint64_t *buf = (uint64_t *) mem.addr();
  TimeIt timer{};
  for(uint64_t i=0; i<count; i+=bs){
    if(bs*8 != f->bread((char *) buf, bs*8))
      break;
    if(i == 0)
      expected = buf[0];
    for(unsigned int j=0; j<bs; j++){
      if((expected != buf[j]))
	{
	  if(prints > 32)
	    break;
	  std::cerr << "error at " << i << " " << j << " rval = 0x" << std::hex << buf[j] << " expected = 0x" << expected << "\n" << std::dec;
	  prints++;
	}
      expected = buf[j] + 1; 
    }
  }
  auto runtime = timer.elapsed();
  auto speed = count * 8.0e-6 / runtime;
  std::cerr << "read " << count*8L << " bytes in " << runtime << " seconds, " << speed << " MB/s\n";
}

int main ( int argc, char **argv )
{
  uint64_t length = 1048576L*16;
  Hififo f2{"/dev/hififo_0_2"};
  Hififo f6{"/dev/hififo_0_6"};
  Hififo f0{"/dev/hififo_0_0"};
  Hififo f4{"/dev/hififo_0_4"};

  cerr << "huge pages\n";
  writer(&f2, length, 1);
  checker(&f6, length, 1);
  cerr << "standard pages\n";
  writer(&f2, length, 0);
  checker(&f6, length, 0);

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
  return 0;
}
