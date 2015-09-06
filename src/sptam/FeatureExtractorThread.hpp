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

#include <thread>
#include <opencv2/features2d/features2d.hpp>

class FeatureExtractorThread
{
  public:

    /**
     * Launches a detection / description process for an image
     * in a separate thread.
     */
    FeatureExtractorThread(
      const cv::Mat& image,
      const cv::FeatureDetector& featureDetector,
      const cv::DescriptorExtractor& descriptorExtractor
    );

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
    const cv::FeatureDetector& featureDetector_;
    const cv::DescriptorExtractor& descriptorExtractor_;

    std::thread featureExtractorThread_;

    cv::Mat descriptors_;
    std::vector<cv::KeyPoint> keyPoints_;

    void Extract();

};
