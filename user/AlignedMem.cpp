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
#include <iostream>
#include <stdexcept>
#include "AlignedMem.h"


using namespace std;

AlignedMem::AlignedMem(size_t size, int use_hp)
{
  len = size;
  cerr << "allocating\n";
  if(use_hp){
    buf = (uint64_t *) mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_POPULATE | MAP_HUGETLB, 1, 0); 
    if(buf != MAP_FAILED){
      return;
    }
    cerr << "falling back to regular pages\n";
  }
  buf = (uint64_t *) mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_PRIVATE | MAP_ANONYMOUS | MAP_POPULATE, 1, 0); 
  if (buf != MAP_FAILED)
    return;
  throw std::bad_alloc();
}

AlignedMem::AlignedMem(size_t size)
{
  AlignedMem{size, 1};
}

AlignedMem::~AlignedMem()
{
  cerr << "deallocating\n";
  munmap(buf, len);
}

void * AlignedMem::addr()
{
  return buf;
}
