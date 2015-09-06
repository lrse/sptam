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

    inline cv::Point3d GetPosition() const
    { return pose_.GetPosition(); }

    inline cv::Vec4d GetOrientation() const
    { return pose_.GetOrientationQuaternion(); }

    inline cv::Matx34d GetProjection() const
    { return calibration_.intrinsic * pose_.GetTransformation(); }

    inline bool CanView(const cv::Point3d& pW) const
    { return frustum_.Contains( pW ); }

  private:

    CameraPose pose_;

    CameraParameters calibration_;

    FrustumCulling frustum_;
};

std::ostream& operator << ( std::ostream& os, const Camera& camera);
