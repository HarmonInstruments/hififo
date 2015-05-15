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

#include <unistd.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <iostream>
#include <stdexcept>
#include <ctime>

#include "Hififo.h"

using namespace std;

#define IOC_INFO 0x10
#define IOC_GET 0x11
#define IOC_PUT 0x12
#define IOC_TIMEOUT 0x13
#define IOC_AVAILABLE 0x14
#define IOC_FPGABUILD 0x15

void Hififo::set_timeout(double timeout)
{
	unsigned long ultimeout = (unsigned long) (1000*timeout);
	if(ioctl(fd, _IO('f', IOC_TIMEOUT), ultimeout) != 0)
		throw std::runtime_error( "hififo set timeout failed" );
}

char * Hififo::get_fpga_build_time()
{
	time_t ts = (time_t) ioctl(fd, _IO('f', IOC_FPGABUILD), 0);
	return asctime(localtime(&ts));
}

Hififo::Hififo(const char * filename)
{
	fd = open(filename, O_RDWR);
	if(fd < 0){
		perror(filename);
		cerr << "fifo_open(" << filename << ") failed\n";
		throw std::runtime_error( "hififo open failed" );
	}
	set_timeout(1.0);
}

Hififo::~Hififo()
{
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

ssize_t Hififo::bread(void * buf, size_t count)
{
	ssize_t rc = read(fd, (char *) buf, count);
	if((size_t) rc != count) {
		std::cerr << "rc = " << rc << std::endl;
		throw std::runtime_error( "hififo read failed" );
	}
	return rc;
}
