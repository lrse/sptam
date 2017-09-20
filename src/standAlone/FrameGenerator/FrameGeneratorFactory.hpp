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

#include "IFrameGenerator.h"
#include "VideoFileFrameGenerator.h"
#include "ListOfFilesFrameGenerator.h"
#include "FileSequenceFrameGenerator.h"

std::unique_ptr<IFrameGenerator> createFrameGenerator(
  const std::string& imagesSource,
  const std::string& imagesSourceType,
  const size_t imageBeginIndex = 0,
  const size_t imageEndIndex = std::numeric_limits<size_t>::max()
){
  if ( imagesSourceType.compare("vid") == 0 ) {
    return std::make_unique<VideoFileFrameGenerator>( imagesSource );
  }
  else if ( imagesSourceType.compare("dir") == 0 ) {
    return std::make_unique<FileSequenceFrameGenerator>( imagesSource, imageBeginIndex, imageEndIndex);
  }
  else if ( imagesSourceType.compare("list" ) == 0) {
    return std::make_unique<ListOfFilesFrameGenerator>( imagesSource );
  }

  throw std::invalid_argument("Images source type is not correct");
}
