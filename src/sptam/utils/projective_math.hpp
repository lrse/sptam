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

#include "opencv2/core/version.hpp"
#if CV_MAJOR_VERSION == 2
  #include <opencv2/core/core.hpp>
  #include <opencv2/calib3d/calib3d.hpp>  // decomposeProjectionMatrix
#elif CV_MAJOR_VERSION == 3
  #include <opencv2/core.hpp>
  #include <opencv2/calib3d.hpp>  // decomposeProjectionMatrix
#endif

#include "cv2eigen.hpp"
#include "eigen_alignment.hpp"

namespace Eigen
{
  typedef Matrix<double, 3, 4> Matrix34d;
}

/**
 * @brief Transform a 2D point in inhomogeneous representation to homogeneous.
 */
inline Eigen::Vector3d toHomo(const Eigen::Vector2d& p)
{
  return Eigen::Vector3d( p[0], p[1], 1 );
}

/**
 * @brief Transform a 3D point in inhomogeneous representation to homogeneous.
 */
inline Eigen::Vector4d toHomo(const Eigen::Vector3d& p)
{
  return Eigen::Vector4d( p[0], p[1], p[2], 1 );
}

/**
 * @brief Transform a 2D point in homogeneous representation to inhomogeneous.
 */
inline Eigen::Vector2d toInHomo(const Eigen::Vector3d& p)
{
  return Eigen::Vector2d(p[0], p[1]) / p[2];
}

/**
 * @brief Transform a 3D point in homogeneous representation to inhomogeneous.
 */
inline Eigen::Vector3d toInHomo(const Eigen::Vector4d& p)
{
  return Eigen::Vector3d(p[0], p[1], p[2]) / p[3];
}

/**
 * @brief Transform a 3D point in homogeneous representation to inhomogeneous.
 */
inline cv::Point3d toInHomo(const cv::Vec4d& p)
{
  return cv::Point3d(p[0], p[1], p[2]) / p[3];
}

inline Eigen::Vector3d projectHomo(const Eigen::Matrix34d& projection, const Eigen::Vector3d& p)
{
  return projection * toHomo( p );
}

/**
 * @brief Project a 3D point to a 2D point, using a projection matrix.
 */
inline cv::Point2d project(const Eigen::Matrix34d& projection, const Eigen::Vector3d& p)
{
  return eigen2cv( toInHomo( projectHomo(projection, p) ) );
}

/**
 * @brief Project a vector of 3D points to a vector of 2D points, using a projection matrix.
 */
inline std::vector<cv::Point2d> project(const Eigen::Matrix34d& projection, const std::aligned_vector<Eigen::Vector3d>& points)
{
  std::vector<cv::Point2d> ret;
  ret.reserve( points.size() );

  for ( const auto& point : points )
    ret.push_back( project(projection, point) );

  return ret;
}

/**
 * @brief Compute 3x4 transformation matrix.
 * @return T = [R|t].
 */
inline Eigen::Matrix34d computeTransformation(const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation)
{
  Eigen::Matrix34d transformation = Eigen::Matrix34d::Identity();

  transformation.block<3, 3>(0, 0) = rotation;
  transformation.block<3, 1>(0, 3) = translation;

  return transformation;
}

/**
 * @brief Compute 3x4 projection matrix.
 * @return P = K[R|t].
 */
inline Eigen::Matrix34d computeProjection(const Eigen::Matrix3d& intrinsic, const Eigen::Matrix3d& rotation, const Eigen::Vector3d& translation)
{
  return intrinsic * computeTransformation(rotation, translation);
}

/**
 * @brief Efficiently compute inverse transformation.
 */
inline Eigen::Matrix34d inverseTransformation(const Eigen::Matrix34d& transformation)
{
  Eigen::Matrix34d inverse = Eigen::Matrix34d::Identity();

  // O = R^t
  inverse.block<3, 3>(0, 0) = transformation.block<3, 3>(0, 0).transpose();

  // p = -R^t * t
  inverse.block<3, 1>(0, 3) = - inverse.block<3, 3>(0, 0) * transformation.block<3, 1>(0, 3);

  return inverse;
}

/*
// K[R|t] = P
inline void decomposeProjection(const cv::Matx34d& projection, cv::Matx33d& intrinsic, cv::Matx33d& rotation, cv::Vec3d& translation)
{
  cv::Vec4d posH;
  cv::decomposeProjectionMatrix(projection, intrinsic, rotation, posH);

  // decompose devuelve la posicion en coordenadas homogeneas.
  // primero la convertimos a inhomogeneas, y luego obtenemos
  // la traslacion
  translation = -rotation * toInHomo( posH );
}

inline cv::Matx33d ComputeFundamentalMat (
  const cv::Matx33d& intrinsicC1, const cv::Matx33d& rotationC1, const cv::Vec3d& translationC1,
  const cv::Matx33d& intrinsicC2, const cv::Matx33d& rotationC2, const cv::Vec3d& translationC2 )
{
  // Tranformacion del mundo a la camara 1
  //~ cv::Matx44d T_C1W = ComputeTransformation44(rotationC1, translationC1);

  // transformacion de la camara 2 al mundo
  //~ cv::Matx44d T_C2W = ComputeTransformation44(rotationC2, translationC2);

  // Compute Tranformation between both cameras (for debugging)
//  cv::Matx44d T_WC1 = T_C1W.inv();
//  cv::Matx44d SE3 = T_C2W * T_WC1;

  // Compute Tranformation between both cameras
  // R21 = R2 * R1.t()
  cv::Matx33d rotation_C2C1 = rotationC2 * rotationC1.t() ;

  // T21 = -R2 * R1.t() * T1 + T2
  cv::Vec3d translation_C2C1 = -rotationC2 * rotationC1.t() * translationC1 + translationC2;

  // Construct [T]x
  double tx = translation_C2C1[0];
  double ty = translation_C2C1[1];
  double tz = translation_C2C1[2];

  cv::Matx33d matCrossProduct(
    0, -tz,  ty,
    tz,  0 , -tx,
    -ty,  tx,  0
  );

  // Compute Essential Matrix ([T]x * R)
  cv::Matx33d essentialMat = matCrossProduct * rotation_C2C1;

  // Compute Fundamental Matrix
  return intrinsicC2.t().inv() * essentialMat * intrinsicC1.inv();
}

// Close Form Fundamental Matrix Computation
inline cv::Matx33d ComputeFundamentalMat(const cv::Matx34d& projectionC1, const cv::Matx34d& projectionC2)
{
  // Get camera Rotation and translation from projection matrix
  cv::Matx33d intrinsicC1;
  cv::Matx33d rotationC1;
  cv::Vec3d translationC1;

  DecomposeProjection(projectionC1, intrinsicC1, rotationC1, translationC1);

  // Get camera Rotation and translation from projection matrix
  cv::Matx33d intrinsicC2;
  cv::Matx33d rotationC2;
  cv::Vec3d translationC2;

  DecomposeProjection(projectionC2, intrinsicC2, rotationC2, translationC2);

  return ComputeFundamentalMat(
    intrinsicC1, rotationC1, translationC1,
    intrinsicC2, rotationC2, translationC2
  );
}*/
