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

#include "Measurement.hpp"

Measurement::Measurement(const Type& type, const Source& source, const cv::KeyPoint& keypoint, const cv::Mat& descriptor)
  : keypoints_({ keypoint }), descriptors_({descriptor}),type_( type ), source_(source)
{
  // assert this is treated as a monocular measurement
  assert( type != Measurement::STEREO );
}

Measurement::Measurement(const Source& source, const cv::KeyPoint& KeyPointLeft, const cv::Mat& descriptorLeft, const cv::KeyPoint& KeyPointRight, const cv::Mat& descriptorRight)
  : keypoints_({ KeyPointLeft, KeyPointRight }), descriptors_({descriptorLeft, descriptorRight}) , type_( Measurement::STEREO ), source_(source)
{ }

Measurement::Measurement(const Type& type, const Source& source, const std::vector<cv::KeyPoint>& keypoints, const std::vector<cv::Mat>& descriptors)
  : keypoints_(keypoints), type_(type), source_(source)
{
  for(const auto& descriptor : descriptors)
    descriptors_.push_back(descriptor); // cv::Mat is just a pointer, data is not being copied
}

Measurement::Measurement(const Measurement& measurement)
  : keypoints_(measurement.GetKeypoints()), type_(measurement.GetType()), source_(measurement.GetSource())
{
  for(const auto& descriptor : measurement.GetDescriptors())
    descriptors_.push_back(descriptor); // cv::Mat is just a pointer, data is not being copied
}
