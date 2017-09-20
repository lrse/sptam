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

#include <vector>
#include <opencv2/opencv.hpp>

class Measurement
{
  public:

    // Possible measurement type
    typedef enum { STEREO, LEFT, RIGHT } Type;

    // Possible measurement sources
    typedef enum { SRC_TRIANGULATION, SRC_TRACKER, SRC_REFIND } Source;

  public:

    Measurement(const Type& type, const Source& source, const cv::KeyPoint& keypoint, const cv::Mat& descriptor);

    Measurement(const Source& source, const cv::KeyPoint& KeyPointLeft, const cv::Mat& descriptorLeft, const cv::KeyPoint& KeyPointRight, const cv::Mat& descriptorRight);

    Measurement(const Type& type, const Source& source, const std::vector<cv::KeyPoint>& keyPoints, const std::vector<cv::Mat>& descriptors);

    Measurement(const Measurement& measurement);

    inline const std::vector<cv::KeyPoint>& GetKeypoints() const
    { return keypoints_; }

    inline const cv::Mat& GetDescriptor() const
    { return descriptors_[0]; }

    inline const std::vector<cv::Mat>& GetDescriptors() const
    { return descriptors_; }

    inline const Type& GetType() const
    { return type_; }

    inline const Source& GetSource() const
    { return source_; }

    const cv::KeyPoint& GetMainKeypoint(void) const
    { return keypoints_[0]; }

  private:

    // Image feature position
    std::vector<cv::KeyPoint> keypoints_;

    // Image feature descriptor
    // Data is not being copied, those cv::Mat are used as pointers. Don't pass temporary data!
    std::vector<cv::Mat> descriptors_;

    Type type_;

    Source source_;
};
