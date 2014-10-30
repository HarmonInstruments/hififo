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
#include "Hififo.h"

using namespace std;

#define IOC_INFO 0x10
#define IOC_GET 0x11
#define IOC_PUT 0x12
#define IOC_TIMEOUT 0x13

#define min(x,y) ((x) > (y) ? (y) : (x))
#define max(x,y) ((x) < (y) ? (y) : (x))

void * Hififo::get_buffer(size_t count)
{
	ssize_t rc = ioctl(fd, _IO('f', IOC_GET), count);
	if(rc < 0)
		throw std::runtime_error("hififo device get buffer timed out");
	return &ringbuf[rc];
}

void Hififo::put_buffer(size_t count)
{
	if(ioctl(fd, _IO('f', IOC_PUT), count) != 0)
		throw std::runtime_error( "hififo device put ioctl failed" );
}

void Hififo::put_all()
{
	if(ioctl(fd, _IO('f', IOC_PUT), 0) != 0)
		throw std::runtime_error( "hififo device put ioctl failed" );
}

void Hififo::set_timeout(double timeout)
{
	if(ioctl(fd, _IO('f', IOC_TIMEOUT), 1000000000L*timeout) != 0)
		throw std::runtime_error( "hififo set timeout failed" );
}

Hififo::Hififo(const char * filename)
{
	fd = open(filename, O_RDWR);
	if(fd < 0){
		perror(filename);
		cerr << "fifo_open(" << filename << ") failed\n";
		throw std::runtime_error( "hififo open failed" );
	}
	ringbuf_size = 8 << 20;
	ringbuf = (uint8_t *) mmap(NULL,
				   ringbuf_size,
				   PROT_READ | PROT_WRITE,
				   MAP_SHARED,
				   fd,
				   0);
        if (ringbuf == MAP_FAILED){
		ringbuf = NULL;
		throw std::bad_alloc();
	}
}

Hififo::~Hififo()
{
	if(ringbuf)
		munmap(ringbuf, ringbuf_size);
	cerr << "closing hififo\n";
	close(fd);
}

ssize_t Hififo::bwrite(const char *buf, size_t count)
{
	ssize_t rc = write(fd, buf, count);
	if((size_t) rc != count)
		throw std::runtime_error( "hififo write failed" );
	return rc;
}

ssize_t Hififo::bread(char * buf, size_t count)
{
	ssize_t rc = read(fd, buf, count);
	if((size_t) rc != count)
		throw std::runtime_error( "hififo read failed" );
	return rc;
}
