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

#include "MotionModel.hpp"

MotionModel::MotionModel(const ros::Time& time, const Eigen::Vector3d& initialPosition, const Eigen::Quaterniond& initialOrientation, const Eigen::Matrix6d& initialCovariance)
  : initialized_( false), last_update_( time )
  , position_( initialPosition ), orientation_( initialOrientation ), poseCovariance_( initialCovariance )
  , linearVelocity_( Eigen::Vector3d::Zero() ), angular_velocity_angle_( 0 ), angular_velocity_axis_(1, 0, 0)
{}

void MotionModel::currentPose(Eigen::Vector3d& currentPosition, Eigen::Quaterniond& currentOrientation, Eigen::Matrix6d& covariance) const
{
  currentPosition = position_;
  currentOrientation = orientation_;
  covariance = poseCovariance_;
}

void MotionModel::predictPose(const ros::Time& time, Eigen::Vector3d& predictedPosition, Eigen::Quaterniond& predictedOrientation, Eigen::Matrix6d& predictionCovariance)
{
  // until the first update dt may not make sense. In any case,
  // the velocity is 0, so just return the initial pose.
  if ( not initialized_ )
  {
    predictedPosition = position_;
    predictedOrientation = orientation_;
    predictionCovariance = poseCovariance_;

    return;
  }

  double dt = (time - last_update_).toSec();
  assert(0 <= dt);

  // Compute predicted position by integrating linear velocity
  predictedPosition = position_ + linearVelocity_ * dt;

  // Compute predicted orientation by integrating angular velocity

  //std::cout << "angle: " << angular_velocity_angle_ * dt << std::endl;
  //std::cout << "axis: " << angular_velocity_axis_ << std::endl;
  Eigen::Quaterniond delta_orientation( Eigen::AngleAxisd( angular_velocity_angle_ * dt, angular_velocity_axis_ ) );

  predictedOrientation = orientation_ * delta_orientation;
  predictedOrientation.normalize();

  predictionCovariance = poseCovariance_;
}

// update the camera state given a new camera pose
void MotionModel::updatePose(const ros::Time& time, const Eigen::Vector3d& newPosition, const Eigen::Quaterniond& newOrientation, const Eigen::Matrix6d& covariance)
{
  if ( initialized_ )
  {
    double dt = (time - last_update_).toSec();
    assert(0 < dt);

    // Compute linear velocity
    //std::cout << "new position: " << newPosition << " " << position_ << " " << dt << std::endl;
    Eigen::Vector3d new_linear_velocity( (newPosition - position_) / dt );

    // compute rotation between q1 and q2: q2 * qInverse( q1 )
    // and save in angle axis representation
    Eigen::Quaterniond delta_orientation_q = newOrientation * orientation_.inverse();
    delta_orientation_q.normalize();

    Eigen::AngleAxisd delta_orientation( delta_orientation_q );

    angular_velocity_axis_ = delta_orientation.axis();
    double ang = delta_orientation.angle();

    /* angle axis can return the rotation in the opposite direction by flipping the axis and giving an angle bigger than PI. in that case
     * the angle is to be interpreted as a negative rotation, thus this correction is needed and also the axis is to be flipped back */
    if (ang > M_PI)
    {
      ang = 2 * M_PI - ang;
      angular_velocity_axis_ *= -1;
    }
    angular_velocity_angle_ = ang / dt;
    //std::cout << "angular velocity (angle): " << delta_orientation.angle() << " " << angular_velocity_angle_ << " " << ang << " " << dt << std::endl;
    //std::cout << "axis: " << angular_velocity_axis_ << std::endl;

    // Update the velocity state variables
    linearVelocity_ = new_linear_velocity;
    //std::cout << "linear velocity: " << linearVelocity_;
  }

  // Update the current state variables

  last_update_ = time;

  position_ = newPosition;
  orientation_ = newOrientation;
  poseCovariance_ = covariance;

  initialized_ = true;
}

void MotionModel::applyCorrection(const Eigen::Matrix4d& corr)
{
  Eigen::Isometry3d correction(corr);
  
  Eigen::Isometry3d current;
  current.linear() = orientation_.toRotationMatrix();
  current.translation() = position_;
  
  current = current * correction; // applying correction
  
  // Resetting the pose
  position_ = current.translation();
  orientation_ = Eigen::Quaterniond(current.linear());
  
  // Rotating velocity vectors
  linearVelocity_ = correction.linear().inverse() * linearVelocity_;
  angular_velocity_axis_ = correction.linear().inverse() * angular_velocity_axis_;
}
