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

#include "Frame.hpp"
#include "RowMatcher.hpp"

#include <memory>

class StereoFrame
{
  public:

    typedef std::unique_ptr<StereoFrame> UniquePtr;

    StereoFrame(
      const CameraPose& cameraPose, const CameraParameters& calibrationLeft,
      const double stereo_baseline, const CameraParameters& calibrationRight,
      const ImageFeatures& imageFeaturesLeft, const ImageFeatures& imageFeaturesRight,
      bool bFixed = false
    );

    StereoFrame(const Frame& frameLeft, const Frame& frameRight, const double stereo_baseline, bool bFixed = false);

    inline void AddMeasurementLeft(const Measurement& measurement)
    { frameLeft_.AddMeasurement( measurement ); }

    inline void AddMeasurementRight(const Measurement& measurement)
    { frameRight_.AddMeasurement( measurement ); }

    inline const std::map<MapPoint*, Measurement>& GetMeasurementsLeft() const
    { return frameLeft_.GetMeasurements(); }

    inline const std::map<MapPoint*, Measurement>& GetMeasurementsRight() const
    { return frameRight_.GetMeasurements(); }

    inline bool GetMeasurementLeft(MapPoint* mapPoint, Measurement& measurement) const
    { return frameLeft_.GetMeasurement(mapPoint, measurement); }

    inline bool GetMeasurementRight(MapPoint* mapPoint, Measurement& measurement) const
    { return frameRight_.GetMeasurement(mapPoint, measurement); }

    std::map<MapPoint *, std::pair<Measurement, Measurement> > GetMeasurementsStereo() const;

    std::map<MapPoint *, Measurement> GetMeasurementsLeftOnly() const;

    std::map<MapPoint *, Measurement> GetMeasurementsRightOnly() const;

    // This is used in Map.cpp to evaluate if a keyframe has the minimum number of measurements
    inline size_t GetNumberOfMeasurements() const
    { return GetMeasurementsLeft().size() + GetMeasurementsRight().size(); }

    /**
     * Try to find correspondences between a cloud of mapPoints
     * and the features detected in the left and right frames
     * (independently).
     */
    void FindMatches(
      const std::vector<cv::Point3d>& points,
      const std::vector<cv::Mat>& descriptors,
      const cv::DescriptorMatcher& descriptorMatcher,
      const size_t matchingNeighborhoodThreshold, const double matchingDistanceThreshold,
      std::vector<MEAS>& measurementsLeft,
      std::vector<MEAS>& measurementsRight
    ) const;

    /**
     * Try to match and triangulate unmatched features from
     * the left and right frames.
     */
    void TriangulatePoints(const RowMatcher& matcher,
      std::vector<cv::Point3d>& points,
      std::vector<cv::Point2d>& featuresLeft, std::vector<cv::Mat>& descriptorsLeft,
      std::vector<cv::Point2d>& featuresRight, std::vector<cv::Mat>& descriptorsRight
    );

    void CreatePoints(const RowMatcher& matcher,
      const std::vector<cv::KeyPoint>& keypoints1, const cv::Mat& descriptors1,
      const std::vector<cv::KeyPoint>& keypoints2, const cv::Mat& descriptors2,
      std::vector<cv::Point3d>& points, std::vector<std::pair<size_t, size_t>>& matches
    );

    void ComputeMapPointsNormals() const;

    size_t RemoveMeasurement(MapPoint* mapPoint);

    size_t RemoveBadPointMeasurements();

    void UpdateCameraPose(const CameraPose& cameraPose);

    int GetId() const
    { return id_; }

    void SetId(int id)
    { id_ = id; }

    inline const cv::Point3d GetPosition() const
    { return frameLeft_.GetPosition(); }

    inline const cv::Vec4d GetOrientation() const
    { return frameLeft_.GetOrientation(); }

    inline const Camera& GetCameraLeft() const
    { return frameLeft_.GetCamera(); }

    inline const Camera& GetCameraRight() const
    { return frameRight_.GetCamera(); }

    inline const CameraPose& GetCameraPose() const
    { return frameLeft_.GetCameraPose(); }

    /**
     * True if Keyframe should remain fixed during bundle adjustment,
     * false if it should be adjusted.
     */
    inline bool isFixed() const
    { return bFixed_; }

    Frame& GetFrameLeft()
    { return frameLeft_; }

    const Frame& GetFrameLeft() const
    { return frameLeft_; }

    Frame& GetFrameRight()
    { return frameRight_; }

    const Frame& GetFrameRight() const
    { return frameRight_; }

private:

  /*** private properties ***/

    Frame frameLeft_, frameRight_;

    // extrinsic stereo parameters
    const double stereo_baseline_;

    // True if StereoFrame is not going to modified by BundleAdjustment
    bool bFixed_;

    // KeyFrame Id
    int id_;

    static CameraPose ComputeRightCameraPose(const CameraPose& leftCameraPose, const double stereo_baseline);
};

std::ostream& operator << ( std::ostream& os, const StereoFrame& keyFrame);
