#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <stdint.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

#define IOC_INFO 0x10

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
  long tmp = ioctl(f->fd, _IO('f',0x11), count);
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
  if(ioctl(f->fd, _IO('f',0x12), count) != 0){
    perror("hififo.c: fifo_read_free() failed");
    return -1;
  }
  return 0;
}

void * fifo_write_get(struct fifodev *f, uint64_t count){
  if(count < f->write_available)
    return f->write_base + f->write_pointer;
  long tmp = ioctl(f->fd, _IO('f',0x13), count);
  if(tmp < 0){
    perror("hififo.c: fifo_write_get() failed");
    return NULL;
  }
  f->write_available = tmp;
  if(tmp < count)
    return NULL;
  return f->write_base + f->write_pointer;
}

uint64_t fifo_write_put(struct fifodev *f, uint64_t count){
  f->write_available -= count;
  f->write_pointer += count;
  if(ioctl(f->fd, _IO('f',0x14), count) != 0){
    perror("hififo.c: fifo_write_put() failed");
    return -1;
  }
  return 0;
}

uint64_t fifo_set_timeout(struct fifodev *f, uint64_t timeout){
  if(ioctl(f->fd, _IO('f',0x15), timeout) != 0){
    perror("hififo.c: fifo_set_timeout() failed");
    return -1;
  }
  return 0;
}

int main ( int argc, char **argv )
{
  uint64_t expected = 0;
  uint64_t count;
  int prints = 0;
  struct fifodev *fifodev = fifo_open("/dev/vna");
  if(fifodev == NULL)
    exit(EXIT_FAILURE);
  fprintf(stderr, "f->read_pointer = %ld\n", fifodev->read_pointer);
  uint64_t *wbuf = fifo_write_get(fifodev, 1048576);
  if(wbuf != NULL)
    for(int i=0; i<131072; i++)
      wbuf[i] = 0xDEAD000000000000 + i;
  fifo_write_put(fifodev, 1048576);
  
  uint64_t * d = fifo_read_get(fifodev, 8192);
  fprintf(stderr, "f->read_pointer = %ld\n", fifodev->read_pointer);
  printf("count = %ld\n", fifodev->read_available);
  if(d != NULL){
    fprintf(stderr, "d[0] = %.16lx\n", d[0]);
    fifo_read_free(fifodev, 8192);
  }
  else
    fprintf(stderr, "d = NULL\n");

  fprintf(stderr, "f->read_pointer = %ld\n", fifodev->read_pointer);
  
  for(long i=0;i<1024UL*1UL*1000UL;i+=count){
    d = fifo_read_get(fifodev, 1*1024);
    count = fifodev->read_available & ~7;
    if(d == NULL){
      printf("failed to read, count = %ld\n", count);
      exit(EXIT_FAILURE);
    }
    for(int j=0; j<count/8; j++){
      if((expected != d[j]) && (prints < 32))
	{
	  printf("error at %lx, %x, rval = 0x%.16lx\n", i, j, d[j]);
	  prints++;
	}
      expected = d[j] + 1; 
    }
    fifo_read_free(fifodev, count);
  }
  printf("read %ld bytes\n", count);

  fifo_close(fifodev);

  return 0;
}
