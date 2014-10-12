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

class Sequencer {
 private:
  Hififo_Write * wf;
  Hififo_Read * rf;
  AlignedMem *amem;
  uint64_t * wbuf;
  uint64_t * rbuf;
  size_t wcount;
  size_t wptr;
  size_t bufsize;
  size_t reads_expected;
 public:
  Sequencer(const char * filename_write, const char * filename_read);
  void append(uint64_t data);
  void wait(uint64_t count);
  void write_req(size_t count, uint32_t address, uint64_t * data);
  void write_single(uint32_t address, uint64_t data);
  void read_req(size_t count, uint32_t address);
  uint64_t * run();
};

