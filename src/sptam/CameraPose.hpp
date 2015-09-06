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

#include <opencv2/core/core.hpp>
#include <eigen3/Eigen/Geometry>

class CameraPose {

  public:

    CameraPose();

    /**
     * Build a camera pose from translation and rotation represented
     * as a 3x3 Matrix.
     */
    CameraPose(const cv::Vec3d& translation, const cv::Matx33d& rotation);

    /**
     * Build a camera pose from a position and an orientation represented
     * as a quaternion.
     */
    CameraPose(const cv::Point3d& position, const cv::Vec4d& qOrientation);

    /**
     * Takes the representation of a point given in World coordinate
     * system (or whichever system the camera pose is relative to), and
     * returns the same point in the cameraPose's reference frame.
     */
    /*inline cv::Point3d FromWorld( cv::Point3d x ) const
    { return rotationMatrix_ * x + cv::Point3d( translation_ ); }*/

    /**
     * Takes the representation of a point given in the cameraPose's
     * reference frame, and returns the same point in World coordinate
     * system (or whichever system the camera pose is relative to).
     */
    inline cv::Point3d ToWorld( cv::Point3d x ) const
    { return orientationMatrix_ * x + cv::Point3d( position_ ); }

    /*inline cv::Vec3d GetTranslation() const
    { return translation_; }*/

    inline cv::Point3d GetPosition() const
    { return position_; }

    inline cv::Matx33d GetOrientationMatrix() const
    { return orientationMatrix_; }

    /*inline const cv::Matx33d& GetRotationMatrix() const
    { return rotation_; }*/

    inline cv::Vec4d GetOrientationQuaternion() const
    { return eigen2cv(orientation_);/*rotationMatrixToQuaternion( orientation_ );*/ }

    /*inline cv::Vec4d GetRotationQuaternion() const
    { return rotationMatrixToQuaternion( rotation_ ); }*/

    cv::Matx34d GetTransformation() const;

    cv::Matx44d GetSE3Matrix() const;

  private:

    cv::Point3d position_;

    Eigen::Quaterniond orientation_;

    //cv::Matx33d orientation_;

    cv::Vec3d translation_;

    //cv::Matx33d rotation_;

    cv::Matx33d orientationMatrix_, rotationMatrix_;

  // Eigen <-> OpenCV quaternion convertion

    inline static cv::Vec4d eigen2cv(const Eigen::Quaterniond& q)
    {
      return cv::Vec4d( q.w(), q.x(), q.y(), q.z() );
    }

    inline static Eigen::Quaterniond cv2eigen(const cv::Vec4d& q)
    {
      return Eigen::Quaterniond(q[0], q[1], q[2], q[3]);
    }
};

std::ostream& operator << (std::ostream& os, const CameraPose& cameraPose);
