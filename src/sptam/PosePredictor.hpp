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

#include <opencv2/core/core.hpp>

/**
 * Abstract interface for pose prediction implementations.
 */
class PosePredictor
{
  public:

    virtual ~PosePredictor() {}

    // Get the current camera pose.
    virtual void CurrentCameraPose(cv::Point3d& currentPosition, cv::Vec4d& currentOrientation) const = 0;

    // Predict the next camera pose.
    virtual void PredictNextCameraPose(cv::Point3d& predictedPosition, cv::Vec4d& predictedOrientation) const = 0;

    // Update the motion model given a new camera pose.
    virtual void UpdateCameraPose(const cv::Point3d& newPosition, const cv::Vec4d& newOrientation) = 0;
};

