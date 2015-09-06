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

/**
 * Class that represents a Viewing Frustum
 */
class FrustumCulling
{
  public:

    FrustumCulling(
      const CameraPose& cameraPose,
      double horizontalFOV, double verticalFOV,
      double nearPlaneDist, double farPlaneDist
    );

    /**
     * Test if a point lies inside the frustum or not
     */
    bool Contains(const cv::Point3d& point) const;

    /**
     * Get the frustum origin
     */
    cv::Point3d GetPosition() const;

    /**
     * Debugging Function is used for daw the frustum with PCL
     */
    void GetFarPlaneCorners(cv::Point3d& bottomLeftCorner, cv::Point3d& bottomRightCorner, cv::Point3d& topLeftCorner, cv::Point3d& topRightCorner);

  private:

    CameraPose cameraPose_;

    cv::Vec4d nearPlane_;
    cv::Vec4d farPlane_;
    cv::Vec4d leftPlane_;
    cv::Vec4d rightPlane_;
    cv::Vec4d topPlane_;
    cv::Vec4d bottomPlane_;

    cv::Point3d bottomLeftFarCorner_;
    cv::Point3d bottomRightFarCorner_;
    cv::Point3d topLeftFarCorner_;
    cv::Point3d topRightFarCorner_;

    void ComputeFrustum(
      double horizontalFOV, double verticalFOV,
      double nearPlaneDist, double farPlaneDist
    );
};
