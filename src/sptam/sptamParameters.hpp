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

#include "opencv2/core/version.hpp"
#if CV_MAJOR_VERSION == 2
  #include <opencv2/features2d/features2d.hpp>
#elif CV_MAJOR_VERSION == 3
  #include <opencv2/features2d.hpp>
#endif

/**
 * Collection of some tuning parameters
 * that are given to sptam
 */
struct Parameters
{
  cv::Ptr<cv::DescriptorMatcher> descriptorMatcher;

  // Matching variables
  int matchingCellSize                    = 30;
  int matchingNeighborhoodThreshold       = 1;
  double matchingDistanceThreshold        = 25.0;
  double epipolarDistanceThreshold        = 0.0;

  // Local Bundle Adjustment parameters
  int nKeyFramesToAdjustByLocal           = 10;
  int maxIterationsLocal                  = 20;

  // Keyframe policy variables
  double minimumTrackedPointsRatio        = 0.9;

  // Algorithm-independent feature extraction parameters
  int nFeatures                           = 0;

  #ifdef USE_LOOPCLOSURE
  std::string loopDetectorVocabulary;
  #endif
};
