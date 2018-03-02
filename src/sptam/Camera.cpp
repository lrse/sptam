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
#include "Camera.hpp"

inline FrustumCulling computeFrustum(const CameraPose& pose, const CameraParameters& calibration)
{
  return FrustumCulling(
    pose.GetPosition(), pose.GetOrientationQuaternion(),
    calibration.horizontalFov(), calibration.verticalFov(),
    calibration.frustumNearPlaneDistance(), calibration.frustumFarPlaneDistance()
  );
}

inline Eigen::Matrix34d computeTransformation(const CameraPose& pose)
{
  return inverseTransformation( computeTransformation(pose.GetOrientationMatrix(), pose.GetPosition()) );
}

Camera::Camera(const CameraPose& pose, const CameraParameters& calibration)
  : pose_( pose ), calibration_( calibration )
  , transformation_( computeTransformation( pose ) )
  , projection_( calibration_.intrinsic() * transformation_ )
  , frustum_( computeFrustum( pose, calibration ) )
{}

void Camera::UpdatePose(const CameraPose& newPose)
{
  pose_ = newPose;

  frustum_ = computeFrustum( pose_, calibration_ );
  transformation_ = computeTransformation( newPose );
  projection_ = calibration_.intrinsic() * transformation_;
}

std::ostream& operator << ( std::ostream& os, const Camera& camera)
{
  return os << camera.GetPose();
}
