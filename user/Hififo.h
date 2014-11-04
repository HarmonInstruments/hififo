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

#pragma once

class Hififo {
private:
	int fd;
	uint8_t *ringbuf;
	size_t ringbuf_size;
protected:
public:
	Hififo(const char * filename);
	~Hififo();
	ssize_t bwrite(const char *buf, size_t count);
	ssize_t bread(char * buf, size_t count);
	void * get_buffer(size_t count);
	void put_buffer(size_t count);
	void put_all();
	void set_timeout(double timeout);
	size_t get_available();
	char * get_fpga_build_time();
};
