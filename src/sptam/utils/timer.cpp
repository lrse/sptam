/**
 * This file is part of S-PTAM.
 *
 * Copyright (C) 2013-2017 Taihú Pire
 * Copyright (C) 2014-2017 Thomas Fischer
 * Copyright (C) 2016-2017 Gastón Castro
 * Copyright (C) 2017 Matias Nitsche
 * For more information see <https://github.com/lrse/sptam>
 *
 * S-PTAM is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * S-PTAM is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with S-PTAM. If not, see <http://www.gnu.org/licenses/>.
 *
 * Authors:  Taihú Pire
 *           Thomas Fischer
 *           Gastón Castro
 *           Matías Nitsche
 *
 * Laboratory of Robotics and Embedded Systems
 * Department of Computer Science
 * Faculty of Exact and Natural Sciences
 * University of Buenos Aires
 */
#include <iomanip>
#include <ios>
#include <cassert>
#include "timer.h"

sptam::Timer::Timer(void) :
  elapsed_seconds(0), started(false)
{

}

void sptam::Timer::start(void)
{
  assert(!started);
  t = clock_t::now();
  started = true;
}

void sptam::Timer::stop(void)
{
  assert(started);
  elapsed_seconds += std::chrono::duration<double, std::milli>(clock_t::now() - t).count() * 1e-3;
  started = false;
}

double sptam::Timer::elapsed(void) const
{
  return elapsed_seconds;
}

double sptam::Timer::now()
{
  return std::chrono::duration_cast<std::chrono::microseconds>(clock_t::now().time_since_epoch()).count() * 1e-6;
}


std::ostream& operator<< (std::ostream& stream, const sptam::Timer& t)
{
  stream << std::setprecision(16) << std::fixed << t.elapsed();
  return stream;
}
