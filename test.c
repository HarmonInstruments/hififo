#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <stdint.h>
#include <sys/mman.h>
#include <sys/ioctl.h>

struct fifodev
{
  int fd;
  uint64_t mmap_size;
  void * mmdata;
  uint64_t read_available;
  uint64_t read_pointer;
  uint64_t read_mask;
  uint64_t write_available;
  void * write_pointer;
};

struct fifodev * fifo_open(char * filename)
{
  struct fifodev *f = calloc(1, sizeof(struct fifodev));
  if(f == NULL)
    goto fail0;
  f->fd = open(filename, O_RDWR);
  if(f->fd < 0){
    perror(filename);
    goto fail1;
  }
  if(ioctl(f->fd, _IOR('f',0x10, sizeof(uint64_t)), &f->mmap_size) != 0){
    perror("mmap size ioctl failed");
    goto fail2;
  }
  f->mmdata = mmap(NULL, f->mmap_size, PROT_READ | PROT_WRITE, MAP_SHARED, f->fd, 0);
  if(f->mmdata == MAP_FAILED) {
    perror("mmap failed");
    goto fail2;
  } 
  f->read_pointer = ioctl(f->fd, _IO('f',0x22), 0);
  if(f->read_pointer < 0) {
    perror("hififo.c: read pointer ioctl() failed");
    goto fail3;
  }
  f->read_mask = (2*1024*1024)-1;
  fprintf(stderr, "f->read_pointer = %ld\n", f->read_pointer);
  return f;
 fail3:
  munmap(f->mmdata, f->mmap_size);
 fail2:
  close(f->fd);
 fail1:
  free(f);
 fail0:
  fprintf(stderr, "fifo_open(%s) failed\n", filename);
  return NULL;
}

void fifo_close(struct fifodev *f){
  munmap(f->mmdata, f->mmap_size);
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
    return f->mmdata + f->read_pointer;
  long tmp = ioctl(f->fd, _IO('f',0x21), count);
  if(tmp < 0){
    perror("hififo.c: fifo_read_get failed");
    return NULL;
  }
  f->read_available = tmp;
  if(tmp < count)
    return NULL;
  return f->mmdata + f->read_pointer;
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
  uint64_t tmp[2];
  if(count < f->write_available)
    return f->write_pointer;
  tmp[0] = count;
  if(ioctl(f->fd, _IOWR('f',0x13, uint64_t[2]), count) != 0){
    perror("hififo.c: fifo_write_get() failed");
    return NULL;
  }
  f->write_available = tmp[0];
  f->write_pointer = f->mmdata + tmp[1];
  return f->write_pointer;
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
  struct fifodev *fifodev = fifo_open("/dev/vna");
  if(fifodev == NULL)
    exit(EXIT_FAILURE);
  
  fprintf(stderr, "attempting mmap reads\n");
  
  for(int i=0; i<16; i++)
    fprintf(stderr, "mmdata[%d] = 0x%.16lx\n", i, ((uint64_t *) fifodev->mmdata)[i]);
  fprintf(stderr, "f->read_pointer = %ld\n", fifodev->read_pointer);
  uint64_t * d = fifo_read_get(fifodev, 8192);
  fprintf(stderr, "f->read_pointer = %ld\n", fifodev->read_pointer);
  uint64_t count; 
  
  printf("count = %ld\n", fifodev->read_available);
  if(d != NULL){
    fprintf(stderr, "d[0] = %.16lx\n", d[0]);
    fifo_read_free(fifodev, 8192);
  }
  else
    fprintf(stderr, "d = NULL");

  fprintf(stderr, "f->read_pointer = %ld\n", fifodev->read_pointer);
  
  for(int i=0;i<1024UL*1000*1000*10;i+=count){
    d = fifo_read_get(fifodev, 3*1024*1024);
    count = fifodev->read_available & ~7;
    if(d == NULL){
      printf("fail, count = %ld\n", count);
      exit(EXIT_FAILURE);
    }
    for(int j=0; j<count/8; j++){
      if(expected != d[j])
	printf("error at %x, %x, rval = 0x%.16lx\n", i, j, d[j]);
      expected = d[j] + 1; 
    }
    fifo_read_free(fifodev, count);
  }
  printf("read %ld bytes\n", count);

  fifo_close(fifodev);

  return 0;
}
