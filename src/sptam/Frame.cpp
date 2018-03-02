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

#include "Frame.hpp"
#include "utils/macros.hpp"
#include "utils/projective_math.hpp"

#if SHOW_PROFILING
#include "utils/timer.h"
#include "utils/log/Profiler.hpp"
#endif

Frame::Frame(const Camera& camera, const ImageFeatures& imageFeatures)
  : camera_( camera ), imageFeatures_( imageFeatures )
{}

#define PROFILE_INTERNAL 1

std::list<std::pair<size_t, size_t> > Frame::FindMatches(const std::aligned_vector<Eigen::Vector3d>& points,
  const std::vector<cv::Mat>& descriptors,
  const cv::DescriptorMatcher& descriptorMatcher,
  const double matchingDistanceThreshold,
  const size_t matchingNeighborhoodThreshold
) const
{
#if SHOW_PROFILING && PROFILE_INTERNAL
  sptam::Timer t_project;
  t_project.start();
#endif

  std::vector<cv::Point2d> featurePredictions = project(camera_.GetProjection(), points);

#if SHOW_PROFILING && PROFILE_INTERNAL
  t_project.stop();
  WriteToLog(" xx FindMatchesFrame-project: ", t_project);
#endif

  return imageFeatures_.FindMatches(
    featurePredictions, descriptors, descriptorMatcher,
    matchingDistanceThreshold, matchingNeighborhoodThreshold
  );
}
