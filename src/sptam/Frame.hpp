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

#include "Camera.hpp"
#include "Measurement.hpp"
#include "ImageFeatures.hpp"

#include <opencv2/core/core.hpp>

class Frame
{
  public:

    Frame(const Camera& camera, const ImageFeatures& imageFeatures);

    inline const std::map<MapPoint*, Measurement>& GetMeasurements() const
    { return measurements_; }

    void AddMeasurement(const Measurement& measurement);

    void AddMeasurement(int meas_idx, const MapPoint& mapPoint, Measurement::Source_t source);

    size_t RemoveMeasurement(MapPoint* mapPoint);

    size_t RemoveBadPointMeasurements();

    bool GetMeasurement(const MapPoint* mapPoint, Measurement& measurement) const;

    inline const cv::Point3d GetPosition() const
    { return camera_.GetPosition(); }

    inline const cv::Vec4d GetOrientation() const
    { return camera_.GetOrientation(); }

    inline const Camera& GetCamera() const
    { return camera_; }

    inline const CameraPose& GetCameraPose() const
    { return camera_.GetPose(); }

    inline cv::Matx34d GetProjection() const
    { return camera_.GetProjection(); }

    inline void UpdateCameraPose(const CameraPose& cameraPose)
    { camera_.UpdatePose( cameraPose ); }

    /**
     * Match a set of 3D points with their respective descriptors
     * to the features in the current frame.
     */
    void FindMatches(
      const std::vector<cv::Point3d>& points,
      const std::vector<cv::Mat>& descriptors,
      const cv::DescriptorMatcher& descriptorMatcher,
      const double matchingDistanceThreshold, const size_t matchingNeighborhoodThreshold,
      std::vector<MEAS>& measurements
    ) const;

    /**
     * Find a feature from thew frame image that matches the given descriptor.
     * A predicted position is provided to make the search easier.
     * If no match is found return false, otherwise return true and fill 'match'
     * with the apropiate values.
     */
    int FindMatch(
      const MapPoint& mapPoint,
      const cv::DescriptorMatcher& descriptorMatcher,
      const double matchingDistanceThreshold,
      const size_t matchingNeighborhoodThreshold
    ) const;

    inline void SetMatchedKeyPoint( size_t index )
    { imageFeatures_.SetMatchedKeyPoint( index ); }

    // TODO: en progreso

    inline void GetUnmatchedKeyPoints(std::vector<cv::KeyPoint>& keyPoints, cv::Mat& descriptors, std::vector<size_t>& indexes) const
    { return imageFeatures_.GetUnmatchedKeyPoints(keyPoints, descriptors, indexes); }

  private:

    std::vector<cv::Point2d> ComputeProjections(const std::vector<cv::Point3d>& points);

  private:

    // the pose of the camera
    Camera camera_;

    // The measurements of map point found on the image
    std::map<MapPoint*, Measurement> measurements_;

    // imageFeatures for matching during refinement
    // It's mutable because it marks matched features
    // to boost matching performance.
    // TODO tal vez deberian ser mutable ciertos CAMPOS de imageFeatures
    // y no todo el objeto
    mutable ImageFeatures imageFeatures_;
};
