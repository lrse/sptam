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

#include "MotionModel.hpp"

inline cv::Vec4d eigen2cv(const Eigen::Quaterniond& q)
{
  return cv::Vec4d( q.w(), q.x(), q.y(), q.z() );
}

inline Eigen::Quaterniond cv2eigen(const cv::Vec4d& q)
{
  return Eigen::Quaterniond(q[0], q[1], q[2], q[3]);
}

MotionModel::MotionModel(const cv::Point3d& initialPosition, const cv::Vec4d& initialOrientation)
  : position_( initialPosition ), linearVelocity_(0, 0, 0)
{
  orientation_ = cv2eigen( initialOrientation );
  angularVelocity_ = cv2eigen( cv::Vec4d(1, 0, 0, 0) );
}

void MotionModel::CurrentCameraPose(cv::Point3d& currentPosition, cv::Vec4d& currentOrientation) const
{
  currentPosition = position_;
  currentOrientation = eigen2cv( orientation_ );
}

void MotionModel::PredictNextCameraPose(cv::Point3d& predictedPosition, cv::Vec4d& predictedOrientation) const
{
  // Compute predicted position by integrating linear velocity

  predictedPosition = position_ + cv::Point3d( linearVelocity_ );

  // Compute predicted orientation by integrating angular velocity

  Eigen::Quaterniond predictedOrientation_e = orientation_ * angularVelocity_;
  predictedOrientation_e.normalize();
  predictedOrientation = eigen2cv( predictedOrientation_e );
}

// update the camera state given a new camera pose
void MotionModel::UpdateCameraPose(const cv::Point3d& newPosition, const cv::Vec4d& newOrientation)
{
  // Compute linear velocity
  cv::Vec3d newLinearVelocity( newPosition - position_ );
  // In order to be robust against fast camera movements linear velocity is smoothed over time
  newLinearVelocity = (newLinearVelocity + linearVelocity_) * 0.5;

  // compute rotation between q1 and q2: q2 * qInverse(q1);
  Eigen::Quaterniond newAngularVelocity = cv2eigen( newOrientation ) * orientation_.inverse();

  // In order to be robust against fast camera movements angular velocity is smoothed over time
  newAngularVelocity = newAngularVelocity.slerp(0.5, angularVelocity_);

  newAngularVelocity.normalize();

  // Update the current state variables

  position_ = newPosition;
  orientation_ = cv2eigen( newOrientation );
  linearVelocity_ = newLinearVelocity;
  angularVelocity_ = newAngularVelocity;
}
