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

#ifndef HIFIFO_H
#define HIFIFO_H

#include "AlignedMem.h"

struct foo {
  int member;
};
 
using namespace std;

#define IOC_INFO 0x10

#define min(x,y) ((x) > (y) ? (y) : (x))
#define max(x,y) ((x) < (y) ? (y) : (x))

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
  ssize_t ioc(void * buf, ssize_t count, int wait, int abort);
public:
  Hififo(const char * filename);
  ~Hififo();
  ssize_t bwrite(const char *buf, size_t count);
  ssize_t bread(char * buf, size_t count);
  ssize_t request(void * buf, ssize_t count);
  ssize_t wait_all();
  ssize_t wait_all_but_one();
  ssize_t abort();
  void init_buffer(size_t size);
};

class Hififo_Read : public Hififo 
{
private:
  void update_request();
public:
  Hififo_Read(const char * filename)
  :Hififo(filename) {}
  size_t get_read_buffer(void ** buffer);
  void put_read_buffer();
};

class Hififo_Write : public Hififo 
{
 private:
  void update_request();
 public:
 Hififo_Write(const char * filename)
   :Hififo(filename) {}
  size_t get_write_buffer(void ** buffer);
  void put_write_buffer();
};

#endif
