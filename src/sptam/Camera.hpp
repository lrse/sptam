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

#include "CameraPose.hpp"
#include "CameraParameters.hpp"
#include "FrustumCulling.hpp"

class Camera
{
  public:

    Camera(const CameraPose& pose, const CameraParameters& calibration);

    void UpdatePose(const CameraPose& newPose);

    inline const CameraPose& GetPose() const
    { return pose_; }

    inline const Eigen::Vector3d& GetPosition() const
    { return pose_.GetPosition(); }

    inline const Eigen::Quaterniond& GetOrientation() const
    { return pose_.GetOrientationQuaternion(); }

    inline const Eigen::Matrix3d& GetIntrinsics() const
    { return calibration_.intrinsic(); }

    inline const Eigen::Matrix34d& GetTransformation() const
    { return transformation_; }

    inline const Eigen::Matrix34d& GetProjection() const
    { return projection_; }

    inline const CameraParameters& GetCalibration() const
    { return calibration_; }

    inline bool CanView(const Eigen::Vector3d& pW) const
    { return frustum_.Contains( pW ); }

  private:

    CameraPose pose_;

    CameraParameters calibration_;

    // transform from world coordinates to camera coordinates.
    Eigen::Matrix34d transformation_;

    // transform from world coordinates to image coordinates.
    Eigen::Matrix34d projection_;

    FrustumCulling frustum_;
};

std::ostream& operator << ( std::ostream& os, const Camera& camera);
