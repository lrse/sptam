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

////////////////////////////////////////////////////////////////////////////////
// Jacobian computation functions. functions with a '2' suffix are the versions
// using my own calculations for what should be the projection derivatives.
// The other functions are the versions given and used by by g2o for BA.

#include <Eigen/Eigenvalues>

namespace Eigen
{
  typedef Matrix<double, 9, 9> Matrix9d;
}

Eigen::Matrix<double,2,6> jacobianXj(const Eigen::Matrix<double,3,4>& camera_transform, const Eigen::Matrix3d& Kcam, const Eigen::Vector3d& point_world);

Eigen::Matrix<double,2,3> jacobianXi(const Eigen::Matrix<double,3,4>& camera_transform, const Eigen::Matrix3d& Kcam, const Eigen::Vector3d& point_world);

//////////////////////////////////////////////////////////////////////////////// 
