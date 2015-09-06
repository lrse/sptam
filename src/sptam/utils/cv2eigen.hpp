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

#include <opencv2/core/eigen.hpp>


inline Eigen::Vector2d cv2eigen(const cv::Point2d& p_cv)
{
  return Eigen::Vector2d(p_cv.x, p_cv.y);
}

inline Eigen::Vector3d cv2eigen(const cv::Point3d& p_cv)
{
  return Eigen::Vector3d(p_cv.x, p_cv.y, p_cv.z);
}

inline Eigen::Vector3d cv2eigen(const cv::Vec3d& v_cv)
{
  return Eigen::Vector3d(v_cv[0], v_cv[1], v_cv[2]);
}

inline Eigen::Quaterniond cv2eigen(const cv::Vec4d& q_cv)
{
  return Eigen::Quaterniond(q_cv[0], q_cv[1], q_cv[2], q_cv[3]);
}
