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

#include "ImageFeatures.hpp"
#include "utils/macros.hpp"

#if SHOW_PROFILING
#include "utils/timer.h"
#include "utils/log/Profiler.hpp"
#endif

#include <functional>
#include <iostream>

#define PROFILE_INTERNAL 1

bool compareResponse(int idx1, int idx2, const std::vector<cv::KeyPoint>& keyPoints)
{
  return keyPoints[ idx2 ].response < keyPoints[ idx1 ].response;
}

ImageFeatures::ImageFeatures(
  const cv::Size& image_size,
  const std::vector<cv::KeyPoint> keyPoints,
  const cv::Mat descriptors,
  const size_t MatchingCellSize)
  : hashed_indexes_(image_size.width, image_size.height, MatchingCellSize, MatchingCellSize)
  , image_size_(image_size)
  , hash_cell_size_( MatchingCellSize )
{
  keyPoints_ = keyPoints;
  descriptors_ = descriptors;

  forn(k, keyPoints_.size())
    hashed_indexes_.insert(keyPoints_[ k ].pt.x, keyPoints_[ k ].pt.y, k);

  matchedKeyPoints_ = std::vector<bool>(keyPoints_.size(), false);
}

ImageFeatures::ImageFeatures(const ImageFeatures& other)
  : matchedKeyPoints_( other.matchedKeyPoints_ )
  , keyPoints_( other.keyPoints_ )
  , hashed_indexes_( other.hashed_indexes_ )
  , image_size_( other.image_size_ )
  , hash_cell_size_( other.hash_cell_size_ )
{
  descriptors_ = other.descriptors_.clone();
}

std::list<std::pair<size_t, size_t> > ImageFeatures::FindMatches(
  const std::vector<cv::Point2d>& featurePredictions,
  const std::vector<cv::Mat>& descriptors,
  const cv::DescriptorMatcher& descriptorMatcher,
  const double matchingDistanceThreshold,
  const size_t matchingNeighborhoodThreshold
) const
{
  std::list<std::pair<size_t, size_t> > measurements;

  #if SHOW_PROFILING && PROFILE_INTERNAL
    sptam::Timer t_match;
    t_match.start();
  #endif

  forn (i, featurePredictions.size())
  {
    const cv::Point2d& point = featurePredictions[ i ];
    cv::Mat descriptor = descriptors[ i ];

    /* aca se filtran los puntos por si caen en la imagen o no, esto puede pasar porque el frustum culling deja pasar puntos que sean vistos por una sola camara */
    if (point.x < 0  or image_size_.width <= point.x or point.y < 0 or image_size_.height <= point.y)
      continue;

    int index = FindMatch(point, descriptor, descriptorMatcher, matchingDistanceThreshold, matchingNeighborhoodThreshold);

    if ( index < 0 )
      continue;

    measurements.push_back({i, index});
  }

  #if SHOW_PROFILING && PROFILE_INTERNAL
    t_match.stop();
    WriteToLog(" xx FindMatchesFrame-match: ", t_match);
  #endif

  return measurements;
}

int ImageFeatures::FindMatch(
  const cv::Point2d& prediction,
  const cv::Mat& descriptor,
  const cv::DescriptorMatcher& descriptorMatcher,
  const double matchingDistanceThreshold,
  const size_t matchingNeighborhoodThreshold
) const
{
  size_t radius = ceil( matchingNeighborhoodThreshold );

  cv::Mat mask(cv::Mat::zeros(1, keyPoints_.size(), CV_8UC1));

  for ( auto idx : hashed_indexes_.getNeighborhood(prediction.x, prediction.y, radius) )
  {
    // distance to query origin (in px)
    double dx = prediction.x - keyPoints_[ idx ].pt.x;
    double dy = prediction.y - keyPoints_[ idx ].pt.y;
    double dst = sqrt(dx*dx + dy*dy);

    // only consider if keypoints is unmatched and
    // if coordinates are within radius (in cellSize)
    if ( not matchedKeyPoints_[ idx ] && dst <= matchingNeighborhoodThreshold * hash_cell_size_)
      mask.at<uchar>(0, idx) = true;
  }

  std::vector<cv::DMatch> matches;
  descriptorMatcher.match(descriptor, descriptors_, matches, mask);

  /*
  std::vector< std::vector<cv::DMatch> > match_candidates;
  descriptorMatcher.knnMatch( descriptor, descriptors_, match_candidates, 2, mask);

  // was found a match?
  if (match_candidates.empty())
    return -1;

  std::vector<cv::DMatch>& matches = match_candidates[0];
  */

  // was found a match?
  if (matches.empty())
    return -1;

  // check if satisfy the matching distance threshold
  if (matches[0].distance > matchingDistanceThreshold)
    return -1;

  // check if the second match is close
//  if (matches.size() == 2) {
//    const double ratio = 1.01; // TODO: check this parameter
//    if(ratio * matches[0].distance >  matches[1].distance)
//      return -1;
//  }

  return matches[0].trainIdx;
}

ImageFeatures::iPair ImageFeatures::GetHash(const cv::Point2d& key, const size_t cellSize) const
{
  return iPair( floor(key.y / (float) cellSize), floor(key.x / (float) cellSize) );
}

void ImageFeatures::GetUnmatchedKeyPoints( std::vector<cv::KeyPoint>& keyPoints,
                                           cv::Mat& descriptors,
                                           std::vector<size_t>& indexes) const
{
  /* se reserva el espacio para el peor caso */
  keyPoints.reserve(keyPoints_.size());
  descriptors.reserve(keyPoints_.size());
  indexes.reserve(keyPoints_.size());

  forn (i, keyPoints_.size()) {
    if( not matchedKeyPoints_[ i ] ) {
      keyPoints.push_back( GetKeypoint( i ) );
      descriptors.push_back( GetDescriptor( i ) );
      indexes.push_back( i );
    }
  }
}
