/**
 * This file is part of S-PTAM.
 *
 * Copyright (C) 2015 Taihú Pire and Thomas Fischer
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
 * Authors:  Taihú Pire <tpire at dc dot uba dot ar>
 *           Thomas Fischer <tfischer at dc dot uba dot ar>
 *
 * Laboratory of Robotics and Embedded Systems
 * Department of Computer Science
 * Faculty of Exact and Natural Sciences
 * University of Buenos Aires
 */

#ifndef PROFILER_HPP_
#define PROFILER_HPP_

#include <list>

#include "Time.hpp"
#include "Logger.hpp"

inline void WriteToLog( const std::string& tag, double start, double end )
{
  std::stringstream message;
  message << std::fixed << GetSeg() << tag << (end - start) << std::endl;

  //~ std::cout << message.str();

  Logger::Write( message.str() );
}

template<typename T>
inline void WriteToLog( const std::string& tag, const T& n )
{
  std::stringstream message;
  message << std::fixed << GetSeg() << tag << n << std::endl;

  //~ std::cout << message.str();

  Logger::Write( message.str() );
}

inline void WriteToLog(const std::string msg, const size_t currentFrameIndex, const cv::Point3d& position, const cv::Matx33d& orientation )
{
  std::stringstream message;

  message << msg << " " << currentFrameIndex << " "
    << orientation(0,0) << " " << orientation(0,1) << " " << orientation(0,2) << " " << position.x << " "
    << orientation(1,0) << " " << orientation(1,1) << " " << orientation(1,2) << " " << position.y << " "
    << orientation(2,0) << " " << orientation(2,1) << " " << orientation(2,2) << " " << position.z << " "
    << std::endl;

  //~ std::cout << message.str();

  Logger::Write( message.str() );
}

template<typename T>
inline void WriteToLog(const std::string tag, const std::list<T>& list )
{
  std::stringstream message;

  message << std::fixed << GetSeg() << tag;

  for ( const auto& elem : list )
    message << " " << elem;

  message << std::endl;

  //~ std::cout << message.str();

  Logger::Write( message.str() );
}

inline void WriteToLog(const std::string msg, const size_t currentFrameIndex, const double timestamp, const cv::Point3d& position, const cv::Matx33d& orientation )
{
  std::stringstream message;

  message << std::fixed << msg << " " << currentFrameIndex << " " << timestamp << " "
    << orientation(0,0) << " " << orientation(0,1) << " " << orientation(0,2) << " " << position.x << " "
    << orientation(1,0) << " " << orientation(1,1) << " " << orientation(1,2) << " " << position.y << " "
    << orientation(2,0) << " " << orientation(2,1) << " " << orientation(2,2) << " " << position.z << " "
    << std::endl;

  //~ std::cout << message.str();

  Logger::Write( message.str() );
}

#endif // PROFILER_HPP_
