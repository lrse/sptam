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

/**
 * Class that represents a Viewing Frustum
 */
class FrustumCulling
{
  public:

    FrustumCulling(
      const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
      double horizontalFOV, double verticalFOV, double nearPlaneDist, double farPlaneDist
    );

    /**
     * Test if a point lies inside the frustum or not
     */
    bool Contains(const Eigen::Vector3d& point) const;

    /**
     * Debugging Function is used for daw the frustum with PCL
     */
    void GetFarPlaneCorners(Eigen::Vector3d& bottomLeftCorner, Eigen::Vector3d& bottomRightCorner, Eigen::Vector3d& topLeftCorner, Eigen::Vector3d& topRightCorner);

  private:

    Eigen::Vector3d position_;
    Eigen::Matrix3d orientation_;

    Eigen::Vector4d nearPlane_;
    Eigen::Vector4d farPlane_;
    Eigen::Vector4d leftPlane_;
    Eigen::Vector4d rightPlane_;
    Eigen::Vector4d topPlane_;
    Eigen::Vector4d bottomPlane_;

    // far plane corners are saved to draw frustum when debugging.
    Eigen::Vector3d fp_bl;
    Eigen::Vector3d fp_br;
    Eigen::Vector3d fp_tl;
    Eigen::Vector3d fp_tr;

    void ComputeFrustum(
      double horizontalFOV, double verticalFOV,
      double nearPlaneDist, double farPlaneDist
    );
};
