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

#include "Timestamps.hpp"

#include <fstream>
#include <sstream>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/thread/thread.hpp> 

#define SECONDS_TO_NANOSECONDS 1000000000

inline size_t word_count(std::istream& is)  // can pass an open std::ifstream() to this if required
{
	size_t c = 0;
	for(std::string w; is >> w; ++c);
	return c;
}

inline size_t word_count(const std::string& str)
{
	std::istringstream iss(str);
	return word_count(iss);
}

ros::Time getSystemTime()
{
  boost::posix_time::ptime t = boost::posix_time::microsec_clock::local_time();
  boost::posix_time::time_duration duration = t - boost::posix_time::from_time_t( 0 );

  return ros::Time( duration.total_seconds(), duration.total_nanoseconds() );
}

Timestamps::Timestamps(const double rate, size_t frame_ini)
  : constant_rate_(true), rate_(rate), next_time_( frame_ini * rate ), last_time_update_( 0 )
{}

Timestamps::Timestamps(const std::string& filename, size_t frame_ini)
  : constant_rate_(false), next_frame_( frame_ini ), last_time_update_( 0 )
{
  std::ifstream file( filename );

  if ( not file.is_open() )
    throw std::invalid_argument("Error opening timestamps file " + filename);
  else
    std::cout << "Parsing timestamps file " + filename << std::endl;

  for(std::string line; std::getline(file, line); )
  {
    size_t n_words = word_count( line );

    std::istringstream linestream( line );

    if ( n_words==1 )
    {
      double timestamp;
      linestream >> timestamp;

      times_.push_back( ros::Time( timestamp ) );
    }
    else if ( n_words==2 )
    {
      size_t sec, nsec;
      linestream >> sec >> nsec;

      times_.push_back( ros::Time(sec, nsec) );
    }
    else
      throw std::range_error("Error parsing timestamps file. Each line should contain one double or two unsigned ints.");

  }

  std::cout << "Loaded " << times_.size() << " timestamps poses" << std::endl;
}

ros::Time Timestamps::getNextWhenReady()
{
  ros::Time next_time = constant_rate_ ? next_time_ : times_.at( next_frame_ );
  ros::Time current_time = getSystemTime();

  #ifndef SINGLE_THREAD
  // Compute time remainder so we don't go too fast, sleep if necessary
  if ( ros::Time(0) != last_time_update_ )
  {
    double elapsed = ( current_time - last_time_update_ ).toSec();
    double cycle = ( next_time - last_time_ ).toSec();
    double to_sleep = cycle - elapsed;

    if ( to_sleep < 0 )
      std::cerr << "WARNING tracking is slower than the camera feed by " << 1+to_sleep << " (s)" << std::endl;
    else
    {
      boost::this_thread::sleep_for( boost::chrono::nanoseconds( (int)(to_sleep * SECONDS_TO_NANOSECONDS) ) );
    }
  }
  #endif

  if ( constant_rate_ )
    next_time_ += ros::Duration( rate_ );
  else
    next_frame_++;

  last_time_ = next_time;
  last_time_update_ = current_time;

  return next_time;
}
