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

#include "ListOfFilesFrameGenerator.h"

#include <fstream>
#include <iostream>

#include "opencv2/core/version.hpp"
#if CV_MAJOR_VERSION == 2
  #include <opencv2/highgui/highgui.hpp>
#elif CV_MAJOR_VERSION == 3
  #include <opencv2/imgcodecs.hpp>
  #include <opencv2/highgui.hpp>
#endif

ListOfFilesFrameGenerator::ListOfFilesFrameGenerator(const std::string& filename)
{
  std::ifstream file( filename );

  std::string line;
  while ( std::getline(file, line) )
    filenames_.push_back( line );

  next_ = filenames_.begin();
}

bool ListOfFilesFrameGenerator::getNextFrame(cv::Mat& frame)
{
  if ( next_ == filenames_.end() )
    return false;

  frame = cv::imread( *next_ );

  if (not frame.data)
    throw std::invalid_argument("Unable to read image: " + *next_);

  ++next_;

  return true;
}
