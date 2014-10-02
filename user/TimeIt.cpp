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

#include <time.h>
#include "TimeIt.h"

using namespace std;

TimeIt::TimeIt(void){
  clock_gettime(CLOCK_REALTIME, &start_time);
}

double TimeIt::elapsed(){
  struct timespec stop_time;
  clock_gettime(CLOCK_REALTIME, &stop_time);
  return (stop_time.tv_sec - start_time.tv_sec) + 1e-9 * (stop_time.tv_nsec - start_time.tv_nsec);  
};
