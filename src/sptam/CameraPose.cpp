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

#include "CameraPose.hpp"
#include "utils/projective_math.hpp"

inline cv::Matx33d eigen2cvmat(const Eigen::Matrix3d& m)
{
  cv::Matx33d ret;

  ret(0, 0) = m(0, 0); ret(0, 1) = m(0, 1); ret(0, 2) = m(0, 2);
  ret(1, 0) = m(1, 0); ret(1, 1) = m(1, 1); ret(1, 2) = m(1, 2);
  ret(2, 0) = m(2, 0); ret(2, 1) = m(2, 1); ret(2, 2) = m(2, 2);

  return ret;;
}

inline Eigen::Matrix3d cvmat2eigen(const cv::Matx33d& m)
{
  Eigen::Matrix3d ret;

  ret(0, 0) = m(0, 0); ret(0, 1) = m(0, 1); ret(0, 2) = m(0, 2);
  ret(1, 0) = m(1, 0); ret(1, 1) = m(1, 1); ret(1, 2) = m(1, 2);
  ret(2, 0) = m(2, 0); ret(2, 1) = m(2, 1); ret(2, 2) = m(2, 2);

  return ret;;
}

CameraPose::CameraPose()
  : position_(0, 0, 0), orientation_(1, 0, 0, 0), translation_(0, 0, 0)
{
  orientationMatrix_ = cv::Matx33d::eye();
  rotationMatrix_ = cv::Matx33d::eye();
}

CameraPose::CameraPose(const cv::Vec3d& translation, const cv::Matx33d& rotation)
  : translation_( translation )
{
  rotationMatrix_ = rotation;

  orientationMatrix_ = rotation.t();

  Eigen::Matrix3d orientationMatrix_eigen = cvmat2eigen( orientationMatrix_ );
  orientation_ = orientationMatrix_eigen;

  position_ = -orientationMatrix_ * translation;
}

CameraPose::CameraPose(const cv::Point3d& position, const cv::Vec4d& qOrientation)
  : position_( position )
{
  orientation_ = cv2eigen( qOrientation );

  Eigen::Matrix3d orientationMatrix_eigen = orientation_.toRotationMatrix();
  orientationMatrix_ = eigen2cvmat( orientationMatrix_eigen );

  rotationMatrix_ = orientationMatrix_.t();

  // t = -R * C where C is the camera position
  translation_ = -rotationMatrix_ * position;
}

cv::Matx34d CameraPose::GetTransformation() const
{
  return ComputeTransformation(rotationMatrix_, translation_);
}

cv::Matx44d CameraPose::GetSE3Matrix() const
{
  return ComputeTransformation44(rotationMatrix_, translation_);
}

std::ostream& operator << (std::ostream& os, const CameraPose& cameraPose)
{
  return os << cameraPose.GetPosition() << cameraPose.GetOrientationQuaternion();
}
