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

#pragma once

#include <list>
#include <eigen3/Eigen/Geometry>
// lexical cast apparently automatically sets the precision for the float string.
#include <boost/lexical_cast.hpp>

#include "Logger.hpp"
#include "../timer.h"
#include "../macros.hpp"

inline std::string printFullPrecision( double x )
{
  return boost::lexical_cast<std::string>( x );
}

namespace std {
  inline std::ostream& lazy_endl( std::ostream& os ){
      os.put('\n');

  #ifdef ENABLE_LOG_FLUSH
      os.flush();
  #endif

      return os;
  }
}

#define STREAM_TO_LOG( msg ) { std::stringstream message; message << msg << std::endl; Logger::Write( message.str() ); }

template<typename T>
inline void WriteToLog(const std::string& tag, const T& n)
{
  std::stringstream message;
  message << printFullPrecision( sptam::Timer::now() ) << tag << n << std::lazy_endl;

  Logger::Write( message.str() );
}

template<typename T, typename U>
inline void writeToLog(const std::string& tag, const T& a, const U& b)
{
  std::stringstream message;
  message << tag << " " << a << " " << b << std::lazy_endl;
  Logger::Write( message.str() );
}

template<typename T, typename U, typename V>
inline void writeToLog(const std::string& tag, const T& a, const U& b, const V& c)
{
  std::stringstream message;
  message << tag << " " << a << " " << b << " " << c << std::lazy_endl;
  Logger::Write( message.str() );
}

template<typename T>
inline void WriteToLog(const std::string tag, const std::list<T>& list)
{
  std::stringstream message;

  message << printFullPrecision( sptam::Timer::now() ) << tag;

  for ( const auto& elem : list )
    message << " " << elem;

  message << std::lazy_endl;

  Logger::Write( message.str() );
}

namespace Eigen { typedef Matrix<double, 6, 6> Matrix6d; }

template<size_t ROWS, size_t COLS>
inline std::ostream& __matToStream__(std::ostream& os, const Eigen::Matrix<double, ROWS, COLS>& matrix)
{
  forn(i, ROWS) forn(j, COLS)
    os << printFullPrecision( matrix(i,j) ) << " ";
  return os;
}

template<size_t DIM>
inline std::ostream& __vecToStream__(std::ostream& os, const Eigen::Matrix<double, DIM, 1>& vec)
{
  __matToStream__<DIM, 1>(os, vec);
  return os;
}

inline std::ostream& __pointToStream__(std::ostream& os, const Eigen::Vector3d& position, const Eigen::Matrix3d& covariance)
{
  __vecToStream__<3>(os, position);
  __matToStream__<3, 3>(os, covariance);
  return os;
}

inline std::ostream& __poseToStream__(std::ostream& os, const Eigen::Vector3d& position, const Eigen::Matrix3d& orientation, const Eigen::Matrix6d& covariance)
{
  os
    << printFullPrecision( orientation(0,0) ) << " " << printFullPrecision( orientation(0,1) ) << " " << printFullPrecision( orientation(0,2) ) << " " << printFullPrecision( position.x() ) << " "
    << printFullPrecision( orientation(1,0) ) << " " << printFullPrecision( orientation(1,1) ) << " " << printFullPrecision( orientation(1,2) ) << " " << printFullPrecision( position.y() ) << " "
    << printFullPrecision( orientation(2,0) ) << " " << printFullPrecision( orientation(2,1) ) << " " << printFullPrecision( orientation(2,2) ) << " " << printFullPrecision( position.z() ) << " ";

  __matToStream__<6, 6>(os, covariance);
  return os;
}

inline std::ostream& __transfToStream__(std::ostream& os, const Eigen::Matrix4d& transf)
{
  __matToStream__<3, 4>(os, transf.block<3, 4>(0, 0));
  return os;
}

inline void writePoseToLog(const std::string& tag, const size_t currentFrameIndex, const Eigen::Vector3d& position, const Eigen::Matrix3d& orientation, const Eigen::Matrix6d& covariance)
{
  std::stringstream message;

  message << tag << " " << currentFrameIndex << " ";
  __poseToStream__( message, position, orientation, covariance );
  message << std::lazy_endl;

  Logger::Write( message.str() );
}

inline void writeTransfToLog(const std::string& tag, const size_t currentFrameIndex, const Eigen::Matrix4d& transf)
{
  std::stringstream message;

  message << tag << " " << currentFrameIndex << " ";
  __transfToStream__( message, transf );
  message << std::lazy_endl;

  Logger::Write( message.str() );
}
