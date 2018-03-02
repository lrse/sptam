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

#include "../sptam/utils/projective_math.hpp"
#include "RectifiedCameraParameters.hpp"

class CameraParameters
{
  public:

    CameraParameters(const Eigen::Matrix3d& intrinsic, size_t image_width, size_t image_height, double frustum_near_plane_distance, double frustum_far_plane_distance, double baseline)
      : intrinsic_( intrinsic ), image_width_( image_width ), image_height_( image_height )
      , horizontal_fov_( computeFOV( intrinsic(0, 0), image_width ) )
      , vertical_fov_( computeFOV( intrinsic(1, 1), image_height ) )
      , frustum_near_plane_distance_( frustum_near_plane_distance )
      , frustum_far_plane_distance_( frustum_far_plane_distance )
      , rectifiedCameraParameters_(baseline, Eigen::Vector2d( intrinsic_(0, 0), intrinsic_(1, 1) ), Eigen::Vector2d( intrinsic_(0, 2), intrinsic_(1, 2) ))
    {
      assert( 0 < image_width_ );
      assert( 0 < image_height_ );
      assert( 0 < frustum_near_plane_distance_ );
      assert( frustum_near_plane_distance_ < frustum_far_plane_distance_ );
      assert( 0 < rectifiedCameraParameters_.baseline );
      assert( intrinsic(0, 0) == intrinsic(1, 1) );
    }

    inline const Eigen::Matrix3d& intrinsic() const
    { return intrinsic_; }

    inline float focalLength() const
    { return intrinsic_(0, 0); }

    inline Eigen::Vector2d focalLengths() const
    { return rectifiedCameraParameters_.focal_length; }

    inline Eigen::Vector2d principalPoint() const
    { return rectifiedCameraParameters_.principal_point; }

    inline size_t imageWidth() const
    { return image_width_; }

    inline size_t imageHeight() const
    { return image_height_; }

    inline double horizontalFov() const
    { return horizontal_fov_; }

    inline double verticalFov() const
    { return vertical_fov_; }

    double frustumNearPlaneDistance() const
    { return frustum_near_plane_distance_; }

    double frustumFarPlaneDistance() const
    { return frustum_far_plane_distance_; }

    double baseline() const
    { return rectifiedCameraParameters_.baseline; }

  private:

    Eigen::Matrix3d intrinsic_;

    size_t image_width_, image_height_;

    double horizontal_fov_, vertical_fov_;

    double frustum_near_plane_distance_, frustum_far_plane_distance_;

    sptam::RectifiedCameraParameters rectifiedCameraParameters_;

  // helper functions

    /**
     * Return the Field Of View angle for a dimention of the camera,
     * given the focal length and size of the image in that dimention.
     */
    inline double computeFOV(const double focal_length, const double image_size)
    {
      return 2 * atan(image_size / (2 * focal_length)) * 180 / M_PI;
    }

  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
