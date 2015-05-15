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

#include "Hififo.h"
#include <vector>

class Sequencer {
private:
	Hififo * wf;
	Hififo * rf;
	std::vector<uint64_t> wbufv;
	std::vector<uint64_t> rbufv;
    	size_t reads_expected;
public:
	Sequencer(const char * filename_write, const char * filename_read);
	~Sequencer();
	void append(uint64_t data);
	void wait(uint64_t count);
	void write_req(size_t count, uint32_t address, uint64_t * data);
	void write_single(uint32_t address, uint64_t data);
	void read_req(size_t count, uint32_t address);
	void run();
	void write (uint32_t address, uint64_t data, uint64_t count);
	uint64_t read(uint32_t address);
	void read_multi(uint32_t address, void *data, uint64_t count);
	std::vector<uint64_t> read_multi(uint32_t address, uint64_t count);
};
