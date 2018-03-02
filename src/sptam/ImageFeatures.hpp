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

#include <math.h>
#include <opencv2/opencv.hpp>
#include "utils/Hash2D.hpp"

class ImageFeatures
{
  public:

    ImageFeatures(const cv::Size& image_size,
      const std::vector<cv::KeyPoint> keyPoints,
      const cv::Mat descriptors,
      const size_t MatchingCellSize);

    ImageFeatures(const ImageFeatures& imageFeatures);

    std::list<std::pair<size_t, size_t> > FindMatches(
      const std::vector<cv::Point2d>& featurePredictions,
      const std::vector<cv::Mat>& descriptors,
      const cv::DescriptorMatcher& descriptorMatcher,
      const double matchingDistanceThreshold,
      const size_t matchingNeighborhoodThreshold
    ) const;

    inline cv::Mat GetDescriptors() const
    { return descriptors_; }

    inline cv::Mat GetDescriptor( size_t index ) const
    { return descriptors_.row( index ); }

    inline const std::vector<cv::KeyPoint>& GetKeypoints() const
    { return keyPoints_;  }

    inline const cv::KeyPoint& GetKeypoint( size_t index ) const
    { return keyPoints_[ index ]; }

    inline void SetMatchedKeyPoint( size_t index ) const
    { matchedKeyPoints_[ index ] = true; }

    void GetUnmatchedKeyPoints(std::vector<cv::KeyPoint>& keyPoints, cv::Mat& descriptors, std::vector<size_t>& indexes) const;

  private:

    mutable std::vector<bool> matchedKeyPoints_;

    cv::Mat descriptors_;

    std::vector<cv::KeyPoint> keyPoints_;

    Hash2D<size_t> hashed_indexes_;

    cv::Size image_size_;

    // we need this because the matching radius parameter is given in hash-cell units.
    size_t hash_cell_size_;

  // helper functions

    typedef std::pair<int, int> iPair;

    iPair GetHash(const cv::Point2d& key, const size_t cellSize) const;

    int FindMatch(
      const cv::Point2d& prediction,
      const cv::Mat& descriptor,
      const cv::DescriptorMatcher& descriptorMatcher,
      const double matchingDistanceThreshold,
      const size_t matchingNeighborhoodThreshold
    ) const;
};
