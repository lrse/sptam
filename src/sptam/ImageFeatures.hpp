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
#pragma once

#include <opencv2/opencv.hpp>

class ImageFeatures
{
  public:

    ImageFeatures(const cv::Mat& image,
      const std::vector<cv::KeyPoint> keyPoints,
      const cv::Mat descriptors,
      const size_t MatchingCellSize);

    int FindMatch(
      const cv::Point2d& prediction,
      const cv::Mat& descriptor,
      const cv::DescriptorMatcher& descriptorMatcher,
      const double matchingDistanceThreshold,
      const size_t matchingNeighborhoodThreshold
    ) const;

    inline cv::Mat GetDescriptor( size_t index ) const
    { return descriptors_.row( index ); }

    inline const cv::KeyPoint& GetKeypoint( size_t index ) const
    { return keyPoints_[ index ]; }

    // TODO esto va a pasar a KeyFrame

    inline void SetMatchedKeyPoint( size_t index )
    { matchedKeyPoints_[ index ] = true; }

    void GetUnmatchedKeyPoints(std::vector<cv::KeyPoint>& keyPoints,cv::Mat& descriptors, std::vector<size_t>& indexes) const;

  private:

    std::vector<bool> matchedKeyPoints_;

    cv::Mat descriptors_;

    std::vector<cv::KeyPoint> keyPoints_;

    // hashed index variables

    size_t MatchingCellSize_;

    size_t nMatchingBucketsX_, nMatchingBucketsY_;
    size_t nCreationBucketsX_, nCreationBucketsY_;

    std::vector< std::vector< std::vector<int> > > hashedMatchingIndexes_;

    // helper functions

    typedef std::pair<int, int> iPair;

    iPair GetHash(const cv::Point2d& key, const size_t cellSize) const;
};
