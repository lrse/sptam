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

#include <thread>
#include <list>

#include "opencv2/core/version.hpp"
#if CV_MAJOR_VERSION == 2
  #include <opencv2/features2d/features2d.hpp>
#elif CV_MAJOR_VERSION == 3
  #include <opencv2/features2d.hpp>
#endif

class FeatureExtractorThread
{
  public:

    /**
     * Launches a detection / description process for an image
     * in a separate thread.
     */
    FeatureExtractorThread(const cv::Mat& image,
      cv::Ptr<cv::FeatureDetector> featureDetector,
      cv::Ptr<cv::DescriptorExtractor> descriptorExtractor
    , size_t nFeatures);

    /**
     * This call blocks until the detection / description process
     * has finished and the internal thread exits.
     */
    inline void WaitUntilFinished()
    { featureExtractorThread_.join(); }

    /**
     * @brief
     *   Get the computed keypoints. It is mandatory to call
     *   WaitUntilFinished() before calling this function.
     */
    inline const std::vector<cv::KeyPoint>& GetKeyPoints() const
    { return keyPoints_; }

    /**
     * @brief
     *   Get the computed descriptors. It is mandatory to call
     *   WaitUntilFinished() before calling this function.
     */
    inline const cv::Mat& GetDescriptors() const
    { return descriptors_; }

  private:

    cv::Mat image_;
    size_t nFeatures_;
    cv::Ptr<cv::FeatureDetector> featureDetector_, featureDetector2_;
    cv::Ptr<cv::DescriptorExtractor> descriptorExtractor_;

    std::thread featureExtractorThread_;

    cv::Mat descriptors_;
    std::vector<cv::KeyPoint> keyPoints_;


    cv::Size grid_size;

    void Extract(void);
    void ExtractGrid(void);
    void quadtreeFilter(const std::list<cv::KeyPoint> &keypoints_in, std::vector<cv::KeyPoint> &keypoints_out);
};
