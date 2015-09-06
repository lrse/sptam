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

#include <opencv2/features2d/features2d.hpp>

/**
 * Matches two sets of descriptors, asuming that the matches
 * will be found on the same pixel row, which makes the match search
 * much more efficient.
 * This is a fair assumption when computing matches from rectified
 * stereo frames, where projected points share the same baseline.
 */
class RowMatcher
{
  public:

    /**
     * 
     * @param rowRange
     *   the range of rows around the row corresponding to the epipolar
     *   line that will be searched for potential matches.
     *   0 is strict, usually 1 is enough.
     *
     * @param maxDistance
     *   the maximum allowed distance between descriptors
     *   to be considered as a match
		 */
    RowMatcher(double maxDistance, cv::Ptr<cv::DescriptorMatcher> descriptorMatcher, double rowRange = 1.0);

    /**
     * matches each query descriptor to a train descriptor in the same
     * row. For robustness, a keypoint in the +-range is also allowed
     * to match. If no match exists, the match index output is -1.
     *
     * @param keypoints1
     *   query keypoints
     *
     * @param descriptors1
     *   query descriptors
     *
     * @param keypoints2
     *   training keypoints
     *
     * @param descriptors2
     *   training descriptors
     *
     * @param matches
     *   resulting match indexes for each query keypoint, which point
     *   to the matched training keypoint. If no match was found,
     *   the index value is -1.
     */
    void match(
      const std::vector<cv::KeyPoint>& keypoints1, const cv::Mat& descriptors1,
      const std::vector<cv::KeyPoint>& keypoints2, const cv::Mat& descriptors2,
      std::vector<cv::DMatch>& matches) const;

    static void match(
      const cv::DescriptorMatcher& matcher, double matchingDistanceThreshold,
      const std::vector<cv::KeyPoint>& keypoints1, const cv::Mat& descriptors1,
      const std::vector<cv::KeyPoint>& keypoints2, const cv::Mat& descriptors2,
      std::vector<cv::DMatch>& matches, double range = 1.0
    );

  private:

    const cv::Ptr<cv::DescriptorMatcher> descriptorMatcher_;

    const double matchingDistanceThreshold_;

    const double range_;
};
