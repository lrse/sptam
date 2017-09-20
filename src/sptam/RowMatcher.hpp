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
     * @param normType
     *   The norm used to measure distance between descriptors.
     *   Suggested values are:
     *    - NORM_L1 or NORM_L2 for SIFT and SURF descriptors
     *    - NORM_HAMMING for ORB, BRISK, and BRIEF
     *    - NORM_HAMMING2 should be used with ORB when WTA_K==3 or 4
     *
     * @param crossCheck
     *   If true, the match() method will only return pairs (i,j)
     *   such that for i-th query descriptor the j-th descriptor
     *   in the matcher’s collection is the nearest and vice versa,
     *   i.e. it will only return consistent pairs.
     *   Such technique usually produces best results with minimal
     *   number of outliers when there are enough matches.
     *   This is alternative to the ratio test, used by D. Lowe
     *   in SIFT paper.
     *   OpenCV doesnt support using masks with crossCheck enabled.
     *   http://answers.opencv.org/question/30670/using-descriptormatcher-with-mask-and-crosscheck/
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
    RowMatcher(double maxDistance, int normType, bool crossCheck = false, double rowRange = 1.0);

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
