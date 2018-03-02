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

#include "match_to_points.hpp"
#include "utils/eigen_alignment.hpp"

#if SHOW_PROFILING
#include "utils/timer.h"
#include "utils/log/Profiler.hpp"
#endif

std::list<Match> matchToPoints(
  const StereoFrame& frame, ConstIterable<sptam::Map::SharedPoint>&& mapPoints,
  const cv::Ptr<cv::DescriptorMatcher> descriptorMatcher,
  const size_t matchingNeighborhoodThreshold,
  const double matchingDistanceThreshold,
  const Measurement::Source source
)
{
#if SHOW_PROFILING
  sptam::Timer t_find;
  t_find.start();
#endif

  std::aligned_vector<Eigen::Vector3d> points;
  std::vector<cv::Mat> descriptors;

  points.reserve( mapPoints.size() );
  descriptors.reserve( mapPoints.size() );

  for ( const auto& mapPoint : mapPoints ) {
    points.push_back( mapPoint->GetPosition() );
    descriptors.push_back( mapPoint->GetDescriptor() );
  }

  std::list<size_t> matchedIndexes;
  std::list<Measurement> measurements;

  frame.FindMatches(source,
    points, descriptors,
    *descriptorMatcher,
    matchingNeighborhoodThreshold,
    matchingDistanceThreshold,
    matchedIndexes, measurements
  );

  std::list<Match> matches;

  size_t src_idx = 0;
  auto it_src = mapPoints.begin();
  auto it_meas = measurements.begin();
  for ( size_t idx : matchedIndexes )
  {
    while( src_idx < idx ) { src_idx++; it_src++; }
    matches.push_back({*it_src, *it_meas});
    it_meas++;
  }

#if SHOW_PROFILING
  t_find.stop();
  WriteToLog(" XX findMatches-find: ", t_find);
#endif

  return matches;
}
