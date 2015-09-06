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

#include "PosePredictor.hpp"
#include <eigen3/Eigen/Geometry>

/**
 * This motion model follows the one proposed in the master thesis of
 * Christof Hoppe: Large-Scale Robotic SLAM through Visual Mapping p.43
 * TODO (Maybe not anymore)
 */
class MotionModel : public PosePredictor
{
  public:

    MotionModel(const cv::Point3d& initialPosition, const cv::Vec4d& initialOrientation);

    // Get the current camera pose.
    void CurrentCameraPose(cv::Point3d& currentPosition, cv::Vec4d& currentOrientation) const override;

    // Predict the next camera pose.
    void PredictNextCameraPose(cv::Point3d& predictedPosition, cv::Vec4d& predictedOrientation) const override;

    // Update the motion model given a new camera pose.
    void UpdateCameraPose(const cv::Point3d& newPosition, const cv::Vec4d& newOrientation) override;

  private:

    cv::Point3d position_;

    Eigen::Quaterniond orientation_;

    cv::Vec3d linearVelocity_;

    Eigen::Quaterniond angularVelocity_;
};
