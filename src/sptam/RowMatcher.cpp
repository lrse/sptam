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

#include "RowMatcher.hpp"
#include "utils/macros.hpp"
#include <iostream>
#include <iterator>     // std::begin, std::end
#include <numeric>      // std::iota
#include <algorithm>    // std::lower_bound, std::upper_bound, std::sort
#include <functional>   // std::bind

bool compare(int a, int b, const std::vector<cv::KeyPoint>& keypoints)
{
  return keypoints[a].pt.y < keypoints[b].pt.y;
}

/* lower_bound uses compare function as (*it < value) */
bool compareLowerBound(int index, double value, const std::vector<cv::KeyPoint>& keypoints)
{
  return keypoints[index].pt.y < value;
}

/* upper_bound uses compare function as (value < *it) */
bool compareUpperBound(double value, int index, const std::vector<cv::KeyPoint>& keypoints)
{
  return value < keypoints[index].pt.y;
}

RowMatcher::RowMatcher(double maxDistance, cv::Ptr<cv::DescriptorMatcher> descriptorMatcher, double rowRange)
  : descriptorMatcher_( descriptorMatcher->clone() ), matchingDistanceThreshold_( maxDistance ), range_( rowRange )
{}

RowMatcher::RowMatcher(double maxDistance, int normType, bool crossCheck, double rowRange)
  : descriptorMatcher_( new cv::BFMatcher(normType, crossCheck) ), matchingDistanceThreshold_( maxDistance ), range_( rowRange )
{}

void RowMatcher::match(
  const std::vector<cv::KeyPoint>& keypoints1, const cv::Mat& descriptors1,
  const std::vector<cv::KeyPoint>& keypoints2, const cv::Mat& descriptors2,
  std::vector<cv::DMatch>& matches
) const
{
  RowMatcher::match( *descriptorMatcher_, matchingDistanceThreshold_, keypoints1, descriptors1, keypoints2, descriptors2, matches, range_ );
}

void RowMatcher::match(
  const cv::DescriptorMatcher& matcher, double matchingDistanceThreshold,
  const std::vector<cv::KeyPoint>& keypoints1, const cv::Mat& descriptors1,
  const std::vector<cv::KeyPoint>& keypoints2, const cv::Mat& descriptors2,
  std::vector<cv::DMatch>& matches, double range
)
{
  // compute a vector of sorted keypoint / descriptor indexes,
  // sorted by row value
  std::vector<int> sorted_indexes_1( keypoints1.size() );
  std::vector<int> sorted_indexes_2( keypoints2.size() );

  // fill the indexes with {0,1,2...}
  std::iota(sorted_indexes_1.begin(), sorted_indexes_1.end(), 0);
  std::iota(sorted_indexes_2.begin(), sorted_indexes_2.end(), 0);

  // sort the indexes by row value
  std::sort(sorted_indexes_1.begin(), sorted_indexes_1.end(), std::bind(compare, std::placeholders::_1, std::placeholders::_2, keypoints1));
  std::sort(sorted_indexes_2.begin(), sorted_indexes_2.end(), std::bind(compare, std::placeholders::_1, std::placeholders::_2, keypoints2));

  // compute the sorted train descriptor matrix
  cv::Mat descriptors2_sorted( descriptors2.rows, descriptors2.cols, descriptors2.type() );

  forn(i, keypoints2.size()) {
    descriptors2_sorted.row( i ) = descriptors2.row( sorted_indexes_2[ i ] );
  }

  // for each query keypoint / descriptor, find a match
  matches.reserve(keypoints1.size());
  forn(i1, keypoints1.size())
  {
    int idx1 = sorted_indexes_1[ i1 ];

    const cv::Mat& query_descriptor = descriptors1.row( idx1 );
    const cv::Point2d query_measurement = keypoints1[ idx1 ].pt;

    // extract potential matches by row value
    // TODO optimize after testing module, min and max should be
    // incremented in each step from the last one

    // get range iterators for potential matches
    auto min = std::lower_bound(std::begin(sorted_indexes_2), std::end(sorted_indexes_2),
      query_measurement.y - range, std::bind(compareLowerBound,  std::placeholders::_1, std::placeholders::_2, keypoints2));
    auto max = std::upper_bound(std::begin(sorted_indexes_2), std::end(sorted_indexes_2),
      query_measurement.y + range, std::bind(compareUpperBound,  std::placeholders::_1, std::placeholders::_2, keypoints2));

    // if there are no descriptors in the specified row range
    // skip to next query point
    if ( not (max - min) )
      continue;

    // get potential descriptors in row range
    cv::Mat mask(cv::Mat::zeros(1, descriptors2.rows, CV_8UC1));
    for(auto it = min; it != max; it++) {
      int valid_index = *it;
      mask.at<uchar>(valid_index) = true;
    }

    std::vector<cv::DMatch> match_candidates;
    matcher.match(query_descriptor, descriptors2, match_candidates, mask);

    // was found a match?
    if (match_candidates.empty())
      continue;

    // check if satisfy the matching distance threshold
    if (match_candidates[0].distance > matchingDistanceThreshold)
      continue;

    match_candidates[0].queryIdx = idx1;

    matches.push_back(match_candidates[0]);
  }
}
