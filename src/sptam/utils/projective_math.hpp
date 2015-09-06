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

#ifndef _PROJECTIVE_MATH_HPP
#define _PROJECTIVE_MATH_HPP

#include <opencv2/core/core.hpp>
#include <opencv2/calib3d/calib3d.hpp>  // decomposeProjectionMatrix

/**
 * Extract the focal length from a intrinsic parameter matrix
 */
inline cv::Point2d getFocalLength(const cv::Matx33d& intrinsic)
{
  return cv::Point2d(intrinsic(0, 0), intrinsic(1, 1));
}

/**
 * Extract the principal point from a intrinsic parameter matrix
 */
inline cv::Point2d getPrincipalPoint(const cv::Matx33d& intrinsic)
{
  return cv::Point2d(intrinsic(0, 2), intrinsic(1, 2));
}

/**
 * Transform a 3D point in inhomogeneous representation to homogeneous
 */
inline cv::Vec4d toHomo(const cv::Point3d& p)
{
  return cv::Vec4d( p.x, p.y, p.z, 1 );
}

/**
 * Transform a 2D point in inhomogeneous representation to homogeneous
 */
inline cv::Vec3d toHomo(const cv::Point2d& p)
{
  return cv::Vec3d( p.x, p.y, 1 );
}

/**
 * Transform a 3D point in homogeneous representation to inhomogeneous
 */
inline cv::Point3d toInHomo(const cv::Vec4d& p)
{
  return cv::Point3d( p[0] / p[3], p[1] / p[3], p[2] / p[3] );
}

/**
 * Transform a 2D point in homogeneous representation to inhomogeneous
 */
inline cv::Point2d toInHomo(const cv::Vec3d& p)
{
  return cv::Point2d( p[0] / p[2], p[1] / p[2] );
}

/**
 * Project a 3D point to a 2D point, using a projection matrix
 */
inline cv::Point2d project(const cv::Matx34d& projection, const cv::Point3d& p)
{
  return toInHomo( projection * toHomo( p ) );
}

/**
 * TYransform a 3D point into another coordinate frame,
 * using a transformation matrix
 */
inline cv::Point3d transform(const cv::Matx44d& transformation, const cv::Point3d& p)
{
  return toInHomo( transformation * toHomo( p ) );
}

// T = [R|t]
inline cv::Matx34d ComputeTransformation(const cv::Matx33d& rotation, const cv::Vec3d& translation)
{
  return cv::Matx34d(
    rotation(0, 0), rotation(0, 1), rotation(0, 2), translation[0],
    rotation(1, 0), rotation(1, 1), rotation(1, 2), translation[1],
    rotation(2, 0), rotation(2, 1), rotation(2, 2), translation[2]
  );
}

// T = [R|t]
inline cv::Matx44d ComputeTransformation44(const cv::Matx33d& rotation, const cv::Vec3d& translation)
{
  return cv::Matx44d(
    rotation(0, 0), rotation(0, 1), rotation(0, 2), translation[0],
    rotation(1, 0), rotation(1, 1), rotation(1, 2), translation[1],
    rotation(2, 0), rotation(2, 1), rotation(2, 2), translation[2],
    0, 0, 0, 1
  );
}

// [R|t] = T
inline void DecomposeTransformation44(const cv::Matx44d T, cv::Matx33d& rotation, cv::Vec3d& translation)
{
  translation = cv::Vec3d( T(0, 3), T(1, 3), T(2, 3) );

  rotation = cv::Matx33d(
    T(0, 0), T(0, 1), T(0, 2),
    T(1, 0), T(1, 1), T(1, 2),
    T(2, 0), T(2, 1), T(2, 2)
  );
}

// P = K[R|t]
inline cv::Matx34d ComputeProjection(const cv::Matx33d& intrinsic, const cv::Matx33d& rotation, const cv::Vec3d& translation)
{
  return intrinsic * ComputeTransformation(rotation, translation);
}

// K[R|t] = P
inline void DecomposeProjection(const cv::Matx34d& projection, cv::Matx33d& intrinsic, cv::Matx33d& rotation, cv::Vec3d& translation)
{
  cv::Vec4d posH;
  cv::decomposeProjectionMatrix(projection, intrinsic, rotation, posH);

  // decompose devuelve la posicion en coordenadas homogeneas.
  // primero la convertimos a inhomogeneas, y luego obtenemos
  // la traslacion
  translation = -rotation * (cv::Vec3d(posH[0], posH[1], posH[2]) / posH[3]);
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
}

/**
 * Return the Field Of View angle for a dimention of the camera,
 * given the focal length and size of the image in that dimention.
 */
inline double computeFOV(const double focal_length, const double image_size)
{
  return 2 * atan(image_size / (2 * focal_length)) * 180 / M_PI;
}

#endif // _PROJECTIVE_MATH_HPP
