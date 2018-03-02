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

// ros::Time
#ifdef BUILDING_ROS
#include <ros/ros.h>
#else
#include "../sptam/utils/time/time.h"
#endif

#include <eigen3/Eigen/Geometry>

namespace Eigen
{
  typedef Matrix<double, 6, 6> Matrix6d;
}

/**
 * Abstract interface for pose prediction implementations.
 */
class PosePredictor
{
  public:

    virtual ~PosePredictor() {}

    // Retrieve the last computed camera pose.
    // TODO use with care!!! This function should not even exists, it should be handled externally
    virtual void currentPose(Eigen::Vector3d& currentPosition, Eigen::Quaterniond& currentOrientation, Eigen::Matrix6d& covariance) const = 0;

    // Predict the next camera pose.
    // TODO can't be const because ekf internally modifies it's state when predicting. Should something there be mutable?
    virtual void predictPose(const ros::Time& time, Eigen::Vector3d& predictedPosition, Eigen::Quaterniond& predictedOrientation, Eigen::Matrix6d& predictionCovariance) /*const*/ = 0;

    // Update the motion model given a new camera pose.
    virtual void updatePose(const ros::Time& time, const Eigen::Vector3d& newPosition, const Eigen::Quaterniond& newOrientation, const Eigen::Matrix6d& covariance) = 0;

    // Apply transformation correction to the model. Note: This method will be called when it happens an abrupt change in the pose (LoopClosing)
    virtual void applyCorrection(const Eigen::Matrix4d& correction) = 0;
};
