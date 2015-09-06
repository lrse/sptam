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

#include "Logger.hpp"
#include <sstream>
#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/date_time/posix_time/posix_time_io.hpp>

Logger::Logger() {
  std::stringstream fileName;
  boost::posix_time::time_facet *facet = new boost::posix_time::time_facet("%Y-%m-%d_%H:%M:%S");
  fileName.imbue(std::locale(std::cout.getloc(), facet));
  fileName << boost::posix_time::second_clock::local_time() << ".log";
  logFile_.open( fileName.str() );
}

Logger Logger::_instance;


void Logger::Write( const std::string& message ) {

  _instance.logFileMutex_.lock();
  _instance.logFile_ << message;
  _instance.logFileMutex_.unlock();
}
