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

#include "TimeIt.h"
#include "AlignedMem.h"
#include "Hififo.h"

using namespace std;

ssize_t Hififo::ioc(void * buf, ssize_t count, int wait, int abort)
{
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

Hififo::Hififo(const char * filename)
{
  fd = open(filename, O_RDWR);
  if(fd < 0){
    perror(filename);
    cerr << "fifo_open(" << filename << ") failed\n";
    throw std::runtime_error( "hififo open failed" );
  }
}

Hififo::~Hififo()
{
  cerr << "closing hififo\n";
  close(fd);
}


ssize_t Hififo::bwrite(const char *buf, size_t count)
{
  ssize_t rc = write(fd, buf, count);
  if((size_t) rc != count){
    std::cerr << "failed to write, attempted " << count << " bytes, retval = " << rc << "\n";
    throw std::runtime_error( "hififo write failed" );
  }
  return rc;
}

ssize_t Hififo::bread(char * buf, size_t count)
{
  ssize_t rc = read(fd, buf, count);
  if((size_t) rc != count){
    std::cerr << "failed to read, attempted " << count << " bytes, retval = " << rc << "\n";
    throw std::runtime_error( "hififo read failed" );
  }
  return rc;
}

ssize_t Hififo::request(void * buf, ssize_t count)
{
  if(count == 0)
    return 0;
  return ioc(buf, count, 1, 0);
}

ssize_t Hififo::wait_all()
{
  return ioc(NULL, 0, 0, 0);
}

ssize_t Hififo::wait_all_but_one()
{
  return ioc(NULL, 0, -1, 0);
}

ssize_t Hififo::abort()
{
  return ioc(NULL, 0, 1, 1);
}

void Hififo::init_buffer(size_t size)
{
  amem = new AlignedMem{size,1};
  half_size = size / 2;
  buf[0] = amem->addr();
  buf[1] = ((char *)buf[0]) + half_size;
  p_in = 0;
  p_out = 0;
  p_req = 0;
}

void Hififo_Read::update_request()
{
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

size_t Hififo_Read::get_read_buffer(void ** buffer)
{
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

void Hififo_Read::put_read_buffer()
{
  p_out ++;
  update_request();
}

void Hififo_Write::update_request()
{
  while((p_req - p_out) < 2){
    TimeIt timer{};
    ssize_t rc = request(buf[p_req & 1], half_size);
    if(rc != half_size){
      cerr << "read timeout";
      break;
    }
    p_req ++;
    cerr << "req " << p_req << " " << p_out << " " << "update_request completed in "\
	 << timer.elapsed() << " seconds\n";
  }
}

size_t Hififo_Write::get_write_buffer(void ** buffer)
{
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

void Hififo_Write::put_write_buffer()
{
  p_out ++;
  update_request();
}

