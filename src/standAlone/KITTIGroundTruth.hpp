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

#include <string>
#include <vector>
#include <eigen3/Eigen/Geometry>

#include "../sptam/PosePredictor.hpp"
#include "../sptam/utils/eigen_alignment.hpp"

class KITTIGroundTruth : public PosePredictor
{
  public:

    KITTIGroundTruth(const std::string& filename);

    // Get the curent camera pose.
    void currentPose(Eigen::Vector3d& currentPosition, Eigen::Quaterniond& currentOrientation, Eigen::Matrix6d& covariance) const override;

    // Predict the next camera pose.
    void predictPose(const ros::Time& time, Eigen::Vector3d& predictedPosition, Eigen::Quaterniond& predictedOrientation, Eigen::Matrix6d& predictionCovariance) override;

    // Update the motion model given a new camera pose.
    void updatePose(const ros::Time& time, const Eigen::Vector3d& newPosition, const Eigen::Quaterniond& newOrientation, const Eigen::Matrix6d& covariance) override;

    // Reset the model given a new camera pose. Note: This method will be called when it happens an abrupt change in the pose (LoopClosing)
    void applyCorrection(const Eigen::Matrix4d& correction) override;

    std::aligned_vector<Eigen::Vector3d> positions_;
    std::aligned_vector<Eigen::Quaterniond> orientations_;

  private:

    mutable size_t currentFrameIndex_;

    Eigen::Vector3d currentPosition_;
    Eigen::Quaterniond currentOrientation_;

  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
