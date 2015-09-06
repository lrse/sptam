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

#include "Map.hpp"
#include "BundleDriver.hpp"
#include "RowMatcher.hpp"

#include <queue>

/**
 * 
 */
class MapMaker
{
  public:

    /**
     * Collection of some tuning parameters
     * that are given to a Mapper
     */
    struct Parameters
    {
      cv::Ptr<cv::DescriptorMatcher> descriptorMatcher;

      // Matching variables
      size_t matchingCellSize;
      size_t matchingNeighborhoodThreshold;      // default: 1
      double matchingDistanceThreshold; // default: 25
      double epipolarDistanceThreshold; // default: 2

      // min linear distance between KeyFrames
      // that triggers a KeyFrame creation.
      double keyFrameDistanceThreshold;

      // ...
      size_t framesBetweenKeyFrames;
    };

    MapMaker(
      sptam::Map& map,
      const CameraParameters& cameraCalibrationLeft,
      const CameraParameters& cameraCalibrationRight,
      const double stereo_baseline,
      const Parameters& params
    );

    // Generate the initial Map with the stereo camera
    bool InitFromStereo(
      const cv::Mat& frameLeft, const cv::Mat& frameRight,
      const cv::Matx33d& intrinsicLeft, const cv::Matx33d& intrinsicRight,
      const CameraPose& cameraPoseLeft, const CameraPose& cameraPoseRight
    );

    /**
     * Add a key-frame to the map. Called by the tracker.
     * The tracker entry point for adding a new keyframe;
     * the tracker thread doesn't want to hang about, so
     * just dumps it on the top of the mapmaker's queue to
     * be dealt with later, and return.
     */
    virtual void AddKeyFrame(StereoFrame::UniquePtr& keyFrame);

    // Is it a good camera pose to add another KeyFrame?
    bool NeedNewKeyFrame(const StereoFrame& kCurrent);

    // Is the camera far away from the nearest KeyFrame (i.e. maybe lost?)
    //bool IsDistanceToNearestKeyFrameExcessive(StereoFrame &kCurrent);

    // interface compliance
    inline void Stop()
    {}

  // Some functions are protected so they are reachable
  // by the threaded version of the MapMaker.
  protected:

    // TODO ver si se puede hacer privado el mapa
    sptam::Map& map_;


    // Queue of newly-made map points to re-find in other KeyFrames
    std::vector<MapPoint*> newPoints_;

  // Thresholds and Parameters

    size_t nKeyFramesToAdjustByLocal_;

  // Maintenance functions:

    // Interrupt Glogal Bundle Adjustment
    void InterruptBA();

    // Peform a local bundle adjustment which only adjusts
    // recently added key-frames
    void BundleAdjustRecent();

    // Perform bundle adjustment on all keyframes, all map points
    void BundleAdjustAll();


    // Get Triangulated MapPoints from a given KeyFrame

    std::vector<MapPoint*> GetNewMapPoints(const StereoFrame& keyFrame);

    // A general data-association update for a single keyframe
    // Do this on a new key-frame when it's passed in by the tracker
    int ReFindInSingleKeyFrame(StereoFrame& keyFrame);

    // When new map points are generated, they're only created from a stereo pair
    // this tries to make additional measurements in other KFs which they might
    // be in.
    int ReFindNewlyMade( const std::vector<StereoFrame*>& keyFrames );

    void RemoveMeasurements( const std::list< std::pair<StereoFrame*, MapPoint*> >& basMeasurements);

    // Erase point and keyframes that were marked as bad
    // during the last bundle adjustment.
    void CleanupMap();

    // get last n KeyFrames
    const std::vector<StereoFrame*> LastKeyFrames(const unsigned int n) const;

  private:

  /*** Private properties ***/

    BundleDriver bundleDriver_;

    CameraParameters cameraCalibrationLeft_;
    CameraParameters cameraCalibrationRight_;

    cv::Ptr<cv::DescriptorMatcher> descriptorMatcher_;

  // Frustum Culling and Camera variables

    double frustumNearPlaneDist_;
    double frustumFarPlaneDist_;

  // Thresholds and Parameters

    size_t maxIterationsGlobal_;
    size_t maxIterationsLocal_;

    //~ double epipolarDistanceThreshold_;
    double matchingDistanceThreshold_;
    double keyFrameDistanceThreshold_;
    double matchingNeighborhood_;

  /*** Private functions ***/

  // Data association functions:

    // Mapmaker's try-to-find-a-point-in-a-keyframe code. This is used to update
    // data association if a bad measurement was detected, or if a point
    // was never searched for in a keyframe in the first place. This operates
    // much like the tracker! So most of the code looks just like in
    // Matching.cpp.
    bool FindMeasurement(Frame& keyFrame, MapPoint& mapPoint);

  // General Maintenance/Utility:

    double KeyFrameLinearDist(const StereoFrame& k1, const StereoFrame& k2);

    StereoFrame* ClosestKeyFrame(const StereoFrame& k);

    std::vector<StereoFrame*> NClosestKeyFrames(StereoFrame& k, unsigned int N);
};


// TODO: ESTA FUNCION NO DEBERIA IR ACA
bool InitFromStereo(sptam::Map& map, StereoFrame& frame, const cv::Mat& imageLeft, const RowMatcher& matcher);
