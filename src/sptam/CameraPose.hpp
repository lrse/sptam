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

#include <eigen3/Eigen/Geometry>

namespace Eigen
{
  typedef Matrix<double, 6, 6> Matrix6d;
}

class CameraPose
{
  public:

    CameraPose();

    /**
     * Build a camera pose from a position and an orientation represented
     * as a quaternion.
     */
    CameraPose(const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation, const Eigen::Matrix6d& covariance);

    /**
     * Takes the representation of a point given in World coordinate
     * system (or whichever system the camera pose is relative to), and
     * returns the same point in the cameraPose's reference frame.
     */
    /*inline cv::Point3d FromWorld( cv::Point3d x ) const
    { return rotationMatrix_ * x + cv::Point3d( translation_ ); }*/

    /**
     * Takes the representation of a point given in the cameraPose's
     * reference frame, and returns the same point in World coordinate
     * system (or whichever system the camera pose is relative to).
     */
    inline Eigen::Vector3d ToWorld( const Eigen::Vector3d& x ) const
    { return orientationMatrix_ * x + position_; }

    inline const Eigen::Vector3d& GetPosition() const
    { return position_; }

    inline const Eigen::Matrix3d& GetOrientationMatrix() const
    { return orientationMatrix_; }

    inline const Eigen::Quaterniond& GetOrientationQuaternion() const
    { return orientation_; }

    inline const Eigen::Matrix6d& covariance() const
    { return covariance_; }

  private:

    Eigen::Vector3d position_;

    Eigen::Quaterniond orientation_;

    Eigen::Matrix3d orientationMatrix_;

    Eigen::Matrix6d covariance_;
};

std::ostream& operator << ( std::ostream& os, const CameraPose& cameraPose);
