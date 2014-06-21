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
};

struct fifodev * fifo_open(char * filename)
{
  struct fifodev *f = malloc(sizeof(struct fifodev));
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
  return f;

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
 * after this call *data will point to a buffer containing data from
 * the device. The return value indicates the actual number of bytes
 * available. Set count to 0 for non blocking, count to the desired
 * number of bytes for blocking. See fifo_set_timeout() for timeout
 */
uint64_t fifo_read_get(struct fifodev *f, void **data, uint64_t count){
  uint64_t tmp[2];
  tmp[0] = count;
  if(ioctl(f->fd, _IOWR('f',0x11, uint64_t[2]), &tmp) != 0){
    perror("read_get ioctl failed");
    return 0;
  }
  *data = f->mmdata + tmp[1];
  return tmp[0];
}

int fifo_read_free(struct fifodev *f, uint64_t count){
  if(ioctl(f->fd, _IOW('f',0x12, uint64_t), &count) != 0){
    perror("read_free ioctl failed");
    return -1;
  }
  return 0;
}

uint64_t fifo_write_get(struct fifodev *f, void *data, uint64_t count){
  return 0;
}

uint64_t fifo_write_put(struct fifodev *f, void *data, uint64_t count){
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

  uint64_t * d;
  uint64_t count = fifo_read_get(fifodev, &d, 8192);
  
  printf("count = %ld\n", count);
  printf("d[0] = %.16lx\n", d[0]);
  fifo_read_free(fifodev, 8192);
  
  //uint64_t *rdata = malloc(1024*1024*8);
  //if(rdata == NULL){
  //  perror("malloc failed\n");
  //  return -1;
  //}
  for(int i=0;i<1000*50;i++){
    count = fifo_read_get(fifodev, &d, 2*1024*1024);
    if(count < 2*1024*1024){
      printf("fail, count = %d\n", count);
      exit(EXIT_FAILURE);
    }
    for(int j=0; j<256*1024; j++){
      if(expected != d[j])
	printf("error at %x, %x, rval = 0x%.16lx\n", i, j, d[j]);
      expected = d[j] + 1; 
    }
    fifo_read_free(fifodev, 2*1024*1024);
  }

  fifo_close(fifodev);

  return 0;
}
