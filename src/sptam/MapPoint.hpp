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

#include <set>
#include <iostream>
#include <opencv2/core/core.hpp>

class MapPoint
{
  public:


    MapPoint(const cv::Point3d& position, const cv::Point3d& normal, const cv::Mat& descriptor);

    inline void SetId(int id)
    { id_ = id; }

    inline int GetId() const
    { return id_; }

    inline const cv::Point3d& GetPosition() const
    { return position_; }

    inline const cv::Point3d& GetNormal() const
    { return normal_; }

    inline void SetNormal(const cv::Point3d& normal)
    { normal_ = normal; }

    inline void updatePosition(const cv::Point3d& new_position)
    { position_ = new_position; }

    inline const cv::Mat& GetDescriptor() const
    { return descriptor_; }

    inline void SetDescriptor(const cv::Mat& descriptor)
    { descriptor.copyTo(descriptor_); }

    // should this be considered a bad point?
    inline bool IsBad() const
    {
      return
        // If not even the 2 original measurements are treated as inliers
        ( measurementCount_ < 2 )
        // If the outlier count is too high
        or ( 20 < outlierCount_ and inlierCount_ < outlierCount_ )
        // If the measurement count is too low
        or ( 20 < projectionCount_ and 10 * measurementCount_ < projectionCount_ );
    }

    inline void IncreaseOutlierCount()/* const **/
    { outlierCount_++; }

    inline void IncreaseInlierCount()/* const **/
    { inlierCount_++; }

    inline void IncreaseProjectionCount()/* const **/
    { projectionCount_++; }

    inline void IncreaseMeasurementCount() const
    { measurementCount_++; }

    inline void DecreaseMeasurementCount() const
    { measurementCount_--; }


    /**
     * @brief GetMeasurementCount is used for plotting and debugging
     * @return
     */
    inline int GetMeasurementCount() const
    { return measurementCount_; }

	private:

    int id_;

    // position in world coordinates.
    cv::Point3d position_;

    // unit vector pointing from the camera position
    // where the feature was seen to the feature position.
    // Assuming the feature lies on a locally planar surface,
    // it is a rough estimate of the surface normal.
    // TODO we don't need a 3D orientation or scale invariant
    // descriptors since we assume the cameras are on a vehicle
    // moving on the ground, but it might be desirable in the future.
    cv::Point3d normal_;

    // descriptor of the projected feature
    cv::Mat descriptor_;

  // This stats are used to evaluate the quality of the mapPoint.

    /**
     * @brief how many times was this point marked as an outlier
     */
    /*mutable */int outlierCount_;

    /**
     * @brief how many times was this point marked as an outlier
     */
    /*mutable */int inlierCount_;

    /**
     * @brief How many times was this point projected onto an image?
     * In other terms, its the number of views whose frustum contains
     * this point.
     */
    /*mutable */int projectionCount_;

    /**
     * @brief how many times was this point matched to a feature?
     * In other terms, its the number of views which have
     * recorded measurements of this point.
     */
    mutable int measurementCount_;


  public:
    // for visualization debugging
    // TODO sacar de aca adentro
    cv::Vec3b color;


};

std::ostream& operator << (std::ostream& os, const MapPoint& mapPoint);
