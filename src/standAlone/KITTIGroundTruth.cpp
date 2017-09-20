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

#include "KITTIGroundTruth.hpp"
#include "../sptam/utils/macros.hpp"

#include <fstream>
#include <iostream>
#include <eigen3/Eigen/Geometry>

KITTIGroundTruth::KITTIGroundTruth(const std::string& filename)
  : currentFrameIndex_(0)
{
  std::ifstream file( filename );

  // TODO: throw exception
  if ( not file.is_open() )
    std::cerr << "Error opening Ground Truth file " << filename << std::endl;

  //read stream line by line
  for(std::string line; std::getline(file, line); )
  {
    std::istringstream linestream( line );

    // read the 12 pose values provided by KITTI
    double values[ 12 ];
    forn(i, 12)
      linestream >> values[ i ];

    Eigen::Vector3d position( values[3], values[7], values[11] );

    positions_.push_back( position );

    Eigen::Matrix3d O33;
    O33(0, 0) = values[0]; O33(0, 1) = values[1]; O33(0, 2) = values[2];
    O33(1, 0) = values[4]; O33(1, 1) = values[5]; O33(1, 2) = values[6];
    O33(2, 0) = values[8]; O33(2, 1) = values[9]; O33(2, 2) = values[10];

    Eigen::Quaterniond orientation_eigen( O33 );
    orientations_.push_back( orientation_eigen );
  }

  std::cout << "Loaded " << positions_.size() << " ground truth poses" << std::endl;

  currentPosition_ = positions_[ currentFrameIndex_ ];
  currentOrientation_ = orientations_[ currentFrameIndex_ ];
}

void KITTIGroundTruth::currentPose(Eigen::Vector3d& currentPosition, Eigen::Quaterniond& currentOrientation, Eigen::Matrix6d& covariance) const
{
  currentPosition = currentPosition_;
  currentOrientation = currentOrientation_;
}

void KITTIGroundTruth::predictPose(const ros::Time& time, Eigen::Vector3d& predictedPosition, Eigen::Quaterniond& predictedOrientation, Eigen::Matrix6d& predictionCovariance)
{
  Eigen::Vector3d oldPosition = positions_[ currentFrameIndex_ ];
  Eigen::Quaterniond oldOrientation = orientations_[ currentFrameIndex_ ];

  Eigen::Vector3d newPosition = positions_[ currentFrameIndex_+1 ];
  Eigen::Quaterniond newOrientation = orientations_[ currentFrameIndex_+1 ];

  // Compute linear velocity

  Eigen::Vector3d linearVelocity( newPosition - oldPosition );

  // Compute Angular velocity

  // compute rotation between q1 and q2: q2 * qInverse(q1);
  Eigen::Quaterniond angularVelocity = newOrientation * oldOrientation.inverse();
  angularVelocity.normalize();

  // Compute predicted position by integrating linear velocity

  predictedPosition = currentPosition_ + linearVelocity;

  // Compute predicted orientation by integrating angular velocity

  predictedOrientation = currentOrientation_ * angularVelocity;
  predictedOrientation.normalize();

  // The counter is increased
  currentFrameIndex_++;
}

void KITTIGroundTruth::updatePose(const ros::Time& time, const Eigen::Vector3d& newPosition, const Eigen::Quaterniond& newOrientation, const Eigen::Matrix6d& covariance)
{
  currentPosition_ = newPosition;
  currentOrientation_ = newOrientation;
}

void KITTIGroundTruth::applyCorrection(const Eigen::Matrix4d& correction)
{
  // there is no need to correct GT information!
}
