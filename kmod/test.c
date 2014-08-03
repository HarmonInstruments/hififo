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
  uint64_t read_size;
  uint64_t write_size;
  void * read_base;
  void * write_base;
  uint64_t read_available;
  uint64_t write_available;
  uint64_t read_pointer;
  uint64_t write_pointer;
  uint64_t read_mask;
  uint64_t write_mask;
};

struct fifodev * fifo_open(char * filename)
{
  uint64_t tmp[8];
  struct fifodev *f = calloc(1, sizeof(struct fifodev));
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
  f->read_size = tmp[0];
  f->write_size = tmp[1];
  f->read_pointer = tmp[2];
  f->write_pointer = tmp[3];
  fprintf(stderr, "read_size = %ld, write_size = %ld\n", f->read_size, f->write_size);
  
  f->read_base = mmap(NULL, 2 * (f->read_size + f->write_size), PROT_READ | PROT_WRITE, MAP_SHARED, f->fd, 0);
  if(f->read_base == MAP_FAILED) {
    perror("mmap failed");
    goto fail2;
  }
  f->write_base = f->read_base + (2 * f->read_size);
  f->read_mask = f->read_size - 1;
  f->write_mask = f->write_size - 1;
  fprintf(stderr, "f->read_pointer = %ld\n", f->read_pointer);
  return f;
  munmap(f->read_base, 2 * (f->read_size + f->write_size));
 fail2:
  close(f->fd);
 fail1:
  free(f);
 fail0:
  fprintf(stderr, "fifo_open(%s) failed\n", filename);
  return NULL;
}

void fifo_close(struct fifodev *f){
  munmap(f->read_base, 2 * (f->read_size + f->write_size));
  close(f->fd);
  free(f);
}

/* 
 * returns a pointer to a buffer containing at least count
 * bytes from the FIFO or returns NULL if insufficient data
 * is available. Updates f->read_available, can 
 */
void * fifo_read_get(struct fifodev *f, uint64_t count){
  if(count < f->read_available)
    return f->read_base + f->read_pointer;
  long tmp = ioctl(f->fd, _IO('f', IOC_GET_TO_PC), count);
  if(tmp < 0){
    perror("hififo.c: fifo_read_get failed");
    return NULL;
  }
  f->read_available = tmp;
  if(tmp < count)
    return NULL;
  return f->read_base + f->read_pointer;
}

int fifo_read_free(struct fifodev *f, uint64_t count){
  f->read_available -= count;
  f->read_pointer += count;
  f->read_pointer &= f->read_mask;
  if(ioctl(f->fd, _IO('f', IOC_PUT_TO_PC), count) != 0){
    perror("hififo.c: fifo_read_free() failed");
    return -1;
  }
  return 0;
}

void * fifo_write_get(struct fifodev *f, uint64_t count){
  if(count < f->write_available)
    return f->write_base + f->write_pointer;
  long tmp = ioctl(f->fd, _IO('f', IOC_GET_FROM_PC), count);
  if(tmp < 0){
    perror("hififo.c: fifo_write_get() failed");
    return NULL;
  }
  f->write_available = tmp;
  //fprintf(stderr, "fifo_write_get: %lx available\n", tmp);
  if(tmp < count)
    return NULL;
  return f->write_base + f->write_pointer;
}

uint64_t fifo_write_put(struct fifodev *f, uint64_t count){
  f->write_available -= count;
  f->write_pointer += count;
  f->write_pointer &= f->write_mask;
  if(ioctl(f->fd, _IO('f', IOC_PUT_FROM_PC), count) != 0){
    perror("hififo.c: fifo_write_put() failed");
    return -1;
  }
  return 0;
}

uint64_t fifo_set_timeout(struct fifodev *f, uint64_t timeout){
  if(ioctl(f->fd, _IO('f', IOC_SET_TIMEOUT), timeout) != 0){
    perror("hififo.c: fifo_set_timeout() failed");
    return -1;
  }
  return 0;
}

void writer(struct fifodev *f, uint64_t count)
{
  uint64_t rcount = 0;
  uint64_t wcount = 0xDEADE00000000000;
  for(uint64_t i=0; i<count; i+=rcount){
    rcount = min(f->write_size/2, count-i);
    uint64_t *wbuf = fifo_write_get(f, rcount);
    //fprintf(stderr, "writer: i = %lx, rcount = %lx, available = %lx, wbuf = %lx\n", i, rcount, f->write_available, (uint64_t) wbuf);
    if(wbuf == NULL){
      fprintf(stderr, "writer failed to get buffer\n");
      return;
    }
    for(uint64_t j=0; j<(rcount>>3); j++)
      wbuf[j] = wcount++;
    fifo_write_put(f, rcount);
  }
};

void checker(struct fifodev *f, uint64_t count)
{
  uint64_t expected = 0;
  uint64_t rcount = 0;
  int prints = 0;
  for(uint64_t i=0; i<count; i+=rcount){
    rcount = min(f->read_size/2, count-i);
    uint64_t * d = fifo_read_get(f, rcount);
    if(d == NULL){
      fprintf(stderr, "failed to read, count = %ld, avaliable = %ld\n", rcount, f->read_available);
      exit(EXIT_FAILURE);
    }
    for(int j=0; j<(rcount>>3); j++){
      if((expected != d[j]))
	{
	  fprintf(stderr, "error at %lx, %x, rval = 0x%.16lx, expected = 0x%.16lx\n", i, j, d[j], expected);
	  prints++;
	}
      expected = d[j] + 1; 
    }
    fifo_read_free(f, rcount);
    //fprintf(stderr, "read %ld bytes\n", rcount);
  }
  fprintf(stderr, "read %ld bytes\n", count);
}

int main ( int argc, char **argv )
{
  uint64_t length = 1048576L*1024L*10L;
  struct fifodev *fifodev = fifo_open("/dev/hififo0");
  if(fifodev == NULL)
    exit(EXIT_FAILURE);
  #pragma omp parallel sections
  {
    #pragma omp section
    {
      writer(fifodev, length);
    }
    #pragma omp section
    {
      checker(fifodev, length);
    }
  }
  fifo_close(fifodev);

  return 0;
}
