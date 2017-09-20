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

#include "FrustumCulling.hpp"

inline Eigen::Vector4d toHomo(const Eigen::Vector3d& p)
{
  return Eigen::Vector4d( p[0], p[1], p[2], 1 );
}

FrustumCulling::FrustumCulling(
  const Eigen::Vector3d& position, const Eigen::Quaterniond& orientation,
  double horizontalFOV, double verticalFOV, double nearPlaneDist, double farPlaneDist
)
  : position_( position ), orientation_( orientation )
{
  ComputeFrustum( horizontalFOV, verticalFOV, nearPlaneDist, farPlaneDist );
}

bool FrustumCulling::Contains(const Eigen::Vector3d& point) const
{
  Eigen::Vector4d pointHomo = toHomo( point );

  return
    (pointHomo.dot (leftPlane_) <= 0) &&
    (pointHomo.dot (rightPlane_) <= 0) &&
    (pointHomo.dot (topPlane_) <= 0) &&
    (pointHomo.dot (bottomPlane_) <= 0) &&
    (pointHomo.dot (nearPlane_) <= 0) &&
    // el plano lejano no se tiene en cuenta para filtrar puntos (se podria omitir su computo)
    (pointHomo.dot (farPlane_) <= 0);
}

void FrustumCulling::GetFarPlaneCorners(Eigen::Vector3d& bottomLeftCorner, Eigen::Vector3d& bottomRightCorner, Eigen::Vector3d& topLeftCorner, Eigen::Vector3d& topRightCorner)
{
  bottomLeftCorner = fp_bl;
  bottomRightCorner = fp_br;
  topLeftCorner = fp_tl;
  topRightCorner = fp_tr;
}

void FrustumCulling::ComputeFrustum(
  double horizontalFOV, double verticalFOV,
  double nearPlaneDist, double farPlaneDist
)
{
  // view vector for the camera  - third column of the orientation matrix
  Eigen::Vector3d view = orientation_.col( 2 );
  // up vector for the camera  - second column of the orientation matix
  Eigen::Vector3d up = -orientation_.col( 1 );
  // right vector for the camera - first column of the orientation matrix
  Eigen::Vector3d right = orientation_.col( 0 );

  float vfov_rad = float (verticalFOV * M_PI / 180); // degrees to radians
  float hfov_rad = float (horizontalFOV * M_PI / 180); // degrees to radians

  float np_h = float (2 * tan (vfov_rad / 2) * nearPlaneDist);  // near plane height
  float np_w = float (2 * tan (hfov_rad / 2) * nearPlaneDist);  // near plane width

  float fp_h = float (2 * tan (vfov_rad / 2) * farPlaneDist);    // far plane height
  float fp_w = float (2 * tan (hfov_rad / 2) * farPlaneDist);    // far plane width

  Eigen::Vector3d fp_c (position_ + view * farPlaneDist);  // far plane center
  fp_tl = Eigen::Vector3d(fp_c + (up * fp_h / 2) - (right * fp_w / 2));  // Top left corner of the far plane
  fp_tr = Eigen::Vector3d(fp_c + (up * fp_h / 2) + (right * fp_w / 2));  // Top right corner of the far plane
  fp_bl = Eigen::Vector3d(fp_c - (up * fp_h / 2) - (right * fp_w / 2));  // Bottom left corner of the far plane
  fp_br = Eigen::Vector3d(fp_c - (up * fp_h / 2) + (right * fp_w / 2));  // Bottom right corner of the far plane

  Eigen::Vector3d np_c (position_ + view * nearPlaneDist);                   // near plane center
  Eigen::Vector3d np_tr (np_c + (up * np_h / 2) + (right * np_w / 2));   // Top right corner of the near plane
  Eigen::Vector3d np_bl (np_c - (up * np_h / 2) - (right * np_w / 2));   // Bottom left corner of the near plane
  Eigen::Vector3d np_br (np_c - (up * np_h / 2) + (right * np_w / 2));   // Bottom right corner of the near plane

  Eigen::Vector3d fp_b_vec = fp_bl - fp_br;
  Eigen::Vector3d fp_normal =  fp_b_vec.cross(fp_tr - fp_br);   // Far plane equation - cross product of the perpendicular edges of the far plane
  farPlane_.head( 3 ) = fp_normal;
  farPlane_[ 3 ] = -fp_c.dot( fp_normal );

  Eigen::Vector3d np_b_vec = np_tr - np_br;
  Eigen::Vector3d np_normal =  np_b_vec.cross(np_bl - np_br);   // Near plane equation - cross product of the perpendicular edges of the near plane
  nearPlane_.head( 3 ) = np_normal;
  nearPlane_[3] = -np_c.dot( np_normal );

  Eigen::Vector3d a (fp_bl - position_);    // Vector connecting the camera and far plane bottom left
  Eigen::Vector3d b (fp_br - position_);    // Vector connecting the camera and far plane bottom right
  Eigen::Vector3d c (fp_tr - position_);    // Vector connecting the camera and far plane top right
  Eigen::Vector3d d (fp_tl - position_);    // Vector connecting the camera and far plane top left

  //                   Frustum and the vectors a, b, c and d. 'position_' is the position of the camera
  //                             _________
  //                           /|       . |
  //                       d  / |   c .   |
  //                         /  | __._____|
  //                        /  /  .      .
  //                 a <---/-/  .    .
  //                      / / .   .  b
  //                     /   .
  //                     .
  //                   T
  //

//  std::cout << "a: " << a << std::endl;
//  std::cout << "d: " << d << std::endl;
//  std::cout << "d: " << T << std::endl;

  Eigen::Vector3d rightPlane_normal = b.cross (c);
  rightPlane_.head( 3 ) = rightPlane_normal;
  rightPlane_[3] = -position_.dot( rightPlane_normal );

  Eigen::Vector3d leftPlane_normal = d.cross (a);
  leftPlane_.head( 3 ) = leftPlane_normal;
  leftPlane_[3] = -position_.dot( leftPlane_normal );

  Eigen::Vector3d topPlane_normal = c.cross (d);
  topPlane_.head( 3 ) = topPlane_normal;
  topPlane_[3] = -position_.dot( topPlane_normal );

  Eigen::Vector3d bottomPlane_normal = a.cross (b);
  bottomPlane_.head( 3 ) = bottomPlane_normal;
  bottomPlane_[3] = -position_.dot( bottomPlane_normal );

//  std::cout << "leftPlane_normal: " << leftPlane_normal << std::endl;

//  std::cout << "nearPlane_:" << std::endl << nearPlane_ << std::endl;
//  std::cout << "farPlane_:" << std::endl << farPlane_ << std::endl;
//  std::cout << "leftPlane_:" << std::endl << leftPlane_ << std::endl;
//  std::cout << "rightPlane_:" << std::endl << rightPlane_ << std::endl;
//  std::cout << "topPlane_:" << std::endl << topPlane_ << std::endl;
//  std::cout << "bottomPlane_:" << std::endl << bottomPlane_ << std::endl;
}
