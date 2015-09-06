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

#include "FrustumCulling.hpp"
#include "utils/projective_math.hpp"

FrustumCulling::FrustumCulling(
  const CameraPose& cameraPose,
  double horizontalFOV, double verticalFOV,
  double nearPlaneDist, double farPlaneDist
)
  : cameraPose_(cameraPose)
{
  ComputeFrustum( horizontalFOV, verticalFOV, nearPlaneDist, farPlaneDist );
}

bool FrustumCulling::Contains(const cv::Point3d& point) const
{
  cv::Vec4d pointHomo = toHomo( point );

  return
    (pointHomo.dot (leftPlane_) <= 0) &&
    (pointHomo.dot (rightPlane_) <= 0) &&
    (pointHomo.dot (topPlane_) <= 0) &&
    (pointHomo.dot (bottomPlane_) <= 0) &&
    (pointHomo.dot (nearPlane_) <= 0) &&
    // el plano lejano no se tiene en cuenta para filtrar puntos (se podria omitir su computo)
    (pointHomo.dot (farPlane_) <= 0);
}

cv::Point3d FrustumCulling::GetPosition() const
{
  return cameraPose_.GetPosition();
}

void FrustumCulling::GetFarPlaneCorners(cv::Point3d& bottomLeftCorner, cv::Point3d& bottomRightCorner, cv::Point3d& topLeftCorner, cv::Point3d& topRightCorner)
{
  bottomLeftCorner = bottomLeftFarCorner_;
  bottomRightCorner = bottomRightFarCorner_;
  topLeftCorner = topLeftFarCorner_;
  topRightCorner = topRightFarCorner_;
}

  /*** private functions ***/

void FrustumCulling::ComputeFrustum(
  double horizontalFOV, double verticalFOV,
  double nearPlaneDist, double farPlaneDist
)
{
  cv::Vec3d position = GetPosition();

  // view vector for the camera  - third column of the orientation matrix
  cv::Vec3d view = cv::Vec3d( cv::Mat( cameraPose_.GetOrientationMatrix().col(2) ) );
  // up vector for the camera  - second column of the orientation matix
  cv::Vec3d up = -cv::Vec3d( cv::Mat( cameraPose_.GetOrientationMatrix().col(1) ) );
  // right vector for the camera - first column of the orientation matrix
  cv::Vec3d right = cv::Vec3d( cv::Mat( cameraPose_.GetOrientationMatrix().col(0) ) );

  cv::Vec4d pl_n; // near plane
  cv::Vec4d pl_f; // far plane
  cv::Vec4d pl_t; // top plane
  cv::Vec4d pl_b; // bottom plane
  cv::Vec4d pl_r; // right plane
  cv::Vec4d pl_l; // left plane

  float vfov_rad = float (verticalFOV * M_PI / 180); // degrees to radians
  float hfov_rad = float (horizontalFOV * M_PI / 180); // degrees to radians

  float np_h = float (2 * tan (vfov_rad / 2) * nearPlaneDist);  // near plane height
  float np_w = float (2 * tan (hfov_rad / 2) * nearPlaneDist);  // near plane width

  float fp_h = float (2 * tan (vfov_rad / 2) * farPlaneDist);    // far plane height
  float fp_w = float (2 * tan (hfov_rad / 2) * farPlaneDist);    // far plane width

  cv::Vec3d fp_c (position + view * farPlaneDist);  // far plane center
  cv::Vec3d fp_tl (fp_c + (up * fp_h / 2) - (right * fp_w / 2));  // Top left corner of the far plane
  cv::Vec3d fp_tr (fp_c + (up * fp_h / 2) + (right * fp_w / 2));  // Top right corner of the far plane
  cv::Vec3d fp_bl (fp_c - (up * fp_h / 2) - (right * fp_w / 2));  // Bottom left corner of the far plane
  cv::Vec3d fp_br (fp_c - (up * fp_h / 2) + (right * fp_w / 2));  // Bottom right corner of the far plane

  cv::Vec3d np_c (position + view * nearPlaneDist);                   // near plane center
  cv::Vec3d np_tr (np_c + (up * np_h / 2) + (right * np_w / 2));   // Top right corner of the near plane
  cv::Vec3d np_bl (np_c - (up * np_h / 2) - (right * np_w / 2));   // Bottom left corner of the near plane
  cv::Vec3d np_br (np_c - (up * np_h / 2) + (right * np_w / 2));   // Bottom right corner of the near plane

  cv::Vec3d fp_b_vec = fp_bl - fp_br;
  cv::Vec3d fp_normal =  fp_b_vec.cross(fp_tr - fp_br);   // Far plane equation - cross product of the perpendicular edges of the far plane
  cv::Mat pl_f_mat(pl_f, false); // represent as matrix (it is no copied)
  cv::Mat(fp_normal).copyTo(pl_f_mat(cv::Rect(0,0,1,3)));
  pl_f(3) = -fp_c.dot(pl_f_mat(cv::Rect(0,0,1,3)));

  cv::Vec3d np_b_vec = np_tr - np_br;
  cv::Vec3d np_normal =  np_b_vec.cross(np_bl - np_br);   // Near plane equation - cross product of the perpendicular edges of the near plane
  cv::Mat pl_n_mat(pl_n, false); // represent as matrix (it is no copied)
  cv::Mat(np_normal).copyTo(pl_n_mat(cv::Rect(0,0,1,3)));
  pl_n(3) = -np_c.dot(pl_n_mat(cv::Rect(0,0,1,3)));

  cv::Vec3d a (fp_bl - position);    // Vector connecting the camera and far plane bottom left
  cv::Vec3d b (fp_br - position);    // Vector connecting the camera and far plane bottom right
  cv::Vec3d c (fp_tr - position);    // Vector connecting the camera and far plane top right
  cv::Vec3d d (fp_tl - position);    // Vector connecting the camera and far plane top left

  //                   Frustum and the vectors a, b, c and d. 'position' is the position of the camera
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

  cv::Mat pl_r_mat(pl_r, false); // represent as matrix (it is no copied)
  cv::Mat pl_l_mat(pl_l, false); // represent as matrix (it is no copied)
  cv::Mat pl_t_mat(pl_t, false); // represent as matrix (it is no copied)
  cv::Mat pl_b_mat(pl_b, false); // represent as matrix (it is no copied)

  cv::Vec3d pl_r_normal = b.cross (c);
  cv::Mat(pl_r_normal).copyTo(pl_r_mat(cv::Rect(0,0,1,3)));

  cv::Vec3d pl_l_normal = d.cross (a);
  cv::Mat(pl_l_normal).copyTo(pl_l_mat(cv::Rect(0,0,1,3)));

  cv::Vec3d pl_t_normal = c.cross (d);
  cv::Mat(pl_t_normal).copyTo(pl_t_mat(cv::Rect(0,0,1,3)));

  cv::Vec3d pl_b_normal = a.cross (b);
  cv::Mat(pl_b_normal).copyTo(pl_b_mat(cv::Rect(0,0,1,3)));

//  std::cout << "pl_l_normal: " << pl_l_normal << std::endl;

  pl_r(3) = -position.dot(pl_r_mat(cv::Rect(0,0,1,3)));
  pl_l(3) = -position.dot(pl_l_mat(cv::Rect(0,0,1,3)));
  pl_t(3) = -position.dot(pl_t_mat(cv::Rect(0,0,1,3)));
  pl_b(3) = -position.dot(pl_b_mat(cv::Rect(0,0,1,3)));

  // save planes
  nearPlane_ = pl_n;
  farPlane_ = pl_f;
  leftPlane_ = pl_l;
  rightPlane_ = pl_r;
  topPlane_ = pl_t;
  bottomPlane_ = pl_b;

  bottomLeftFarCorner_ = fp_bl;
  bottomRightFarCorner_ = fp_br;
  topLeftFarCorner_ = fp_tl;
  topRightFarCorner_ = fp_tr;

//  std::cout << "pl_n:" << std::endl << pl_n << std::endl;
//  std::cout << "pl_f:" << std::endl << pl_f << std::endl;
//  std::cout << "pl_l:" << std::endl << pl_l << std::endl;
//  std::cout << "pl_r:" << std::endl << pl_r << std::endl;
//  std::cout << "pl_t:" << std::endl << pl_t << std::endl;
//  std::cout << "pl_b:" << std::endl << pl_b << std::endl;
}
