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

#include <set>
#include <iostream>
#include <eigen3/Eigen/Geometry>

#include <boost/thread/shared_mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/thread/lock_types.hpp>

#include "opencv2/core/version.hpp"
#if CV_MAJOR_VERSION == 2
  #include <opencv2/core/core.hpp>
#elif CV_MAJOR_VERSION == 3
  #include <opencv2/core.hpp>
#endif

#define INITIAL_POINT_COVARIANCE Eigen::Matrix3d::Identity() * 1e-4

class MapPoint
{
  public:

    MapPoint(const Eigen::Vector3d& position, const Eigen::Vector3d& normal, const cv::Mat& descriptor, const Eigen::Matrix3d& covariance);

    MapPoint(const MapPoint& mapPoint);

    inline Eigen::Vector3d GetPosition() const
    {
      boost::shared_lock<boost::shared_mutex> lock(mpoint_mutex_);
      return position_;
    }

    inline Eigen::Vector3d GetNormal() const
    {
      boost::shared_lock<boost::shared_mutex> lock(mpoint_mutex_);
      return normal_;
    }

    inline void updateNormal(const Eigen::Vector3d& normal)
    {
      boost::unique_lock<boost::shared_mutex> lock(mpoint_mutex_);
      normal_ = normal;
    }

    inline void updatePosition(const Eigen::Vector3d& new_position)
    {
      boost::unique_lock<boost::shared_mutex> lock(mpoint_mutex_);
      position_ = new_position;
    }

    inline const cv::Mat GetDescriptor() const
    {
      boost::shared_lock<boost::shared_mutex> lock(mpoint_mutex_);
      return descriptor_.clone();
    }

    inline void updateDescriptor(const cv::Mat& descriptor)
    {
      boost::unique_lock<boost::shared_mutex> lock(mpoint_mutex_);
      descriptor.copyTo(descriptor_);
    }

    const Eigen::Matrix3d covariance() const
    {
      boost::shared_lock<boost::shared_mutex> lock(mpoint_mutex_);
      return covariance_;
    }

    // Statistics update operations.

    // should this be considered a bad point?
    inline bool IsBad() const
    {
      boost::shared_lock<boost::shared_mutex> lock(mpoint_mutex_);
      //std::cout << "nMeasurements: " << mapPoint.measurementCount_ << std::endl;
      //std::cout << "projectionCount: " << mapPoint.projectionCount_ << std::endl;
      //std::cout << "inliers: " << mapPoint.inlierCount_ << std::endl;
      //std::cout << "outliers: " << mapPoint.outlierCount_ << std::endl;

      return
        // If it has no measurement
        ( measurementCount_ == 0 )
        // If the outlier count is too high
        or ( 20 < outlierCount_ and inlierCount_ < outlierCount_ )
        // If the measurement count is too low
        or ( 20 < projectionCount_ and 10 * measurementCount_ < projectionCount_ );
    }

    inline void IncreaseOutlierCount()/* const **/
    {
      boost::unique_lock<boost::shared_mutex> lock(mpoint_mutex_);
      outlierCount_++;
    }

    inline void IncreaseInlierCount()/* const **/
    {
      boost::unique_lock<boost::shared_mutex> lock(mpoint_mutex_);
      inlierCount_++;
    }

    inline void IncreaseProjectionCount()/* const **/
    {
      boost::unique_lock<boost::shared_mutex> lock(mpoint_mutex_);
      projectionCount_++;
    }

    inline void IncreaseMeasurementCount()/* const **/
    {
      boost::unique_lock<boost::shared_mutex> lock(mpoint_mutex_);
      measurementCount_++;
    }


    /** NOTE: these are not locked since they are to be used from the main thread which is the same as the tracking thread */

    inline cv::Vec3d getColor() const
    { return color_; }

    inline void setColor(cv::Vec3b& color)
    { color_ = color; }

  private:

    mutable boost::shared_mutex mpoint_mutex_;

    // position in world coordinates.
    Eigen::Vector3d position_;

    Eigen::Matrix3d covariance_;

    // unit vector pointing from the camera position
    // where the feature was seen to the feature position.
    // Assuming the feature lies on a locally planar surface,
    // it is a rough estimate of the surface normal.
    Eigen::Vector3d normal_;

    // descriptor of the projected feature
    cv::Mat descriptor_;

    // This stats are used to evaluate the quality of the mapPoint.

    /**
     * @brief how many times was this point marked as an outlier
     */
    /*mutable */size_t outlierCount_;

    /**
     * @brief how many times was this point marked as an inlier
     */
    /*mutable */size_t inlierCount_;

    /**
     * @brief How many times was this point projected onto an image?
     * In other terms, its the number of views whose frustum contains
     * this point.
     */
    /*mutable */size_t projectionCount_;

    /**
     * @brief How many times was this point find in an image?
     * Note that this include the measurements from the tracking phase
     * and the keyframes.
     */
    /*mutable */size_t measurementCount_;

    cv::Vec3b color_;

    //friend class sptam::Map;
};
