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

#ifndef __STEREO_FRAME_MATCHER_HPP__
#define __STEREO_FRAME_MATCHER_HPP__

#include <vector>

#include "opencv2/features2d/features2d.hpp"
#include "../Map.hpp"

/* Stereo Descriptor Match: Represents a feature visible by all 4 frames.
 * 4-way match between two stereo frames
 * frames 1 and 2 correspond to the first stereo frame
 * frames 3 and 4 correspond to the second stereo frame*/
struct SDMatch{

  SDMatch(const cv::DMatch& m1vs2_, const cv::DMatch& m3vs4_, const cv::DMatch& m1vs3_, const cv::DMatch& m2vs4_)
    : m1vs2(m1vs2_), m3vs4(m3vs4_), m1vs3(m1vs3_), m2vs4(m2vs4_) {}

  cv::DMatch m1vs2;

  cv::DMatch m3vs4;

  cv::DMatch m1vs3;

  cv::DMatch m2vs4;
};

/*void drawStereoMatches(const sptam::Map::KeyFrame&, const sptam::Map::KeyFrame&, const std::vector<SDMatch>&, cv::Mat&);*/

class StereoMatcher
{
  public:

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
     * @param maxDistance
     *   the maximum allowed distance between descriptors
     *   to be considered as a match
     */
    StereoMatcher(double maxDistance, int normType, bool crossCheck=false);

    /* Matches between frames from the same stereo must be provided */
    void match(const sptam::Map::KeyFrame& stereo_frame1, const sptam::Map::KeyFrame& stereo_frame2,
               std::vector<SDMatch>& matches) const;

    static void match(const cv::DescriptorMatcher& matcher, double matchingDistanceThreshold,
                      const cv::Mat& descriptors1, const cv::Mat& descriptors2, const std::vector<cv::DMatch>& matches12,
                      const cv::Mat& descriptors3, const cv::Mat& descriptors4, const std::vector<cv::DMatch>& matches34,
                      std::vector<SDMatch>& matches);

    static void drawStereoMatches(const sptam::Map::SharedKeyFrame&, const sptam::Map::SharedKeyFrame&,
                                  const std::vector<SDMatch>&, cv::Mat&);

  private:

    const cv::Ptr<cv::DescriptorMatcher> matcher_;

    const double matchingDistanceThreshold_;
};

#endif // __STEREO_FRAME_MATCHER_HPP__
