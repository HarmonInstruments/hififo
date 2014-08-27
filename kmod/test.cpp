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

struct fifodev
{
  int fd;
};

struct fifodev * fifo_open(const char * filename)
{
  uint64_t tmp[8];
  struct fifodev *f = (struct fifodev *) calloc(1, sizeof(struct fifodev));
  if(f == NULL)
    goto fail0;
  f->fd = open(filename, O_RDWR);
  if(f->fd < 0){
    perror(filename);
    goto fail1;
  }
  if(ioctl(f->fd, _IOR('f', IOC_INFO, uint64_t[8]), tmp) != 0){
    perror("device info ioctl failed");
    goto fail2;
  }
  return f;
 fail2:
  close(f->fd);
 fail1:
  free(f);
 fail0:
  cerr << "fifo_open(" << filename << ") failed\n";
  //fprintf(stderr, "fifo_open(%s) failed\n", filename);
  return NULL;
}

void fifo_close(struct fifodev *f){
  //munmap(f->read_base, 2 * (f->read_size + f->write_size));
  close(f->fd);
  free(f);
}

uint64_t fifo_set_timeout(struct fifodev *f, uint64_t timeout){
  if(ioctl(f->fd, _IO('f', IOC_SET_TIMEOUT), timeout) != 0){
    perror("hififo.c: fifo_set_timeout() failed");
    return -1;
  }
  return 0;
}

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
    //cerr << "deallocating\n";
    munmap(buf, len);
  }
  void *addr(){
    return buf;
  }
};

class Hififo {
public:
  struct fifodev *fifo;
  Hififo(const char * filename) {
    fifo = fifo_open(filename);
    if(fifo == NULL)
      throw;
  }
  ~Hififo() {
    cerr << "closing hififo\n";
    fifo_close(fifo);
  }
  int bwrite(const char *buf, size_t count){
    ssize_t rc = write(fifo->fd, buf, count);
    if((size_t) rc != count){
      std::cerr << "failed to write, attempted " << count << " bytes, retval = " << rc << "\n";
      throw;
    }
    return rc;
  }
  ssize_t bread(char * buf, size_t count){
    ssize_t rc = read(fifo->fd, buf, count);
    if((size_t) rc != count){
      std::cerr << "failed to read, attempted " << count << " bytes, retval = " << rc << "\n";
      throw;
    }
    return rc;
  }
};

void writer(Hififo *f, size_t count, int use_hp)
{
  size_t bs = 1024*512*16;
  uint64_t wcount = 0xDEADE00000000000;
  AlignedMem mem = {bs*8, use_hp};
  uint64_t *wbuf = (uint64_t *) mem.addr();
  for(size_t j=0; j<bs; j++)
    wbuf[j] = wcount++;
  TimeIt timer{};
  for(uint64_t i=0; i<count; i+=bs){
    f->bwrite((const char *) wbuf, bs*8);
    for(size_t j=0; j<bs; j++)
      wbuf[j] = wcount++;
  }
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
    f->bread((char *) buf, bs*8);
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

  return 0;
}
