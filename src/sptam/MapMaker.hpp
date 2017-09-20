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

#include <Eigen/StdVector>
#include "Map.hpp"
#include "Match.hpp"
#include "BundleDriver.hpp"
#include "RowMatcher.hpp"
#include "sptamParameters.hpp"
#include "utils/fixed_queue.hpp"

#ifdef USE_LOOPCLOSURE
#include "loopclosing/LoopClosing.hpp"
#endif

// Minimum number of measurements required for a keyframe
// to be considered "good". If the number of measurements drops
// below this threshold, the keyframe is erased from the map.
// TODO This should be a parameter
// I put this here because, I am lazy, sorry :( (thomas)
#define MIN_NUM_MEAS 10

/**
 *
 */
class MapMaker
{
  public:

    MapMaker(sptam::Map& map, const Parameters& params);

    /**
     * Add a key-frame to the map. Called by the tracker.
     * The tracker entry point for adding a new keyframe;
     * the tracker thread doesn't want to hang about, so
     * just dumps it on the top of the mapmaker's queue to
     * be dealt with later, and return.
     */
    virtual sptam::Map::SharedKeyFrame AddKeyFrame(const StereoFrame& frame, /*const */std::list<Match>& measurements);

    void addStereoPoints(/*const */sptam::Map::SharedKeyFrame& keyFrame, const std::vector<MapPoint,Eigen::aligned_allocator<MapPoint>>& points, const std::vector<Measurement>& measurements);

    #ifdef USE_LOOPCLOSURE
    void setLoopClosing(std::shared_ptr<LoopClosing>& lc)
    {loopclosing_ = lc;}
    #endif

    size_t getKFsToAdjustByLocal()
    {return params_.nKeyFramesToAdjustByLocal;}


    /**
     * Dummy function in sequential mode
     */
    virtual void Stop(){;}

  // Get Triangulated MapPoints from a given KeyFrame
  std::list<sptam::Map::SharedPoint> getPointsCreatedBy(const sptam::Map::SharedKeyFrame& keyFrame);

  // When new map points are generated, they're only created from a stereo pair
  // this tries to make additional measurements in other KFs which they might
  // be in. It returns the number of newly found measurements.
  size_t ReFind(Iterable<sptam::Map::SharedKeyFrame>&& keyFrames, Iterable<sptam::Map::SharedPoint>&& new_points);

  // Some functions are protected so they are reachable
  // by the threaded version of the MapMaker.
  protected:

    sptam::Map& map_;

    #ifdef USE_LOOPCLOSURE
    /* Loop Closure service, will notify him when a new key frame its added. */
    std::shared_ptr<LoopClosing> loopclosing_;
    #endif

    RowMatcher rowMatcher_;

    typedef fixed_queue< sptam::Map::SharedKeyFrame > KeyFrameCache;

    // list of keyframes to be adjusted by the LBA and used to refind measurements from new created points.
    std::list< sptam::Map::SharedKeyFrame > LBA_keyframes_window_; // Gaston: LoopClosure safe usage windows uses this member variable for sincronization

  // Maintenance functions:

    // Interrupt Glogal Bundle Adjustment
    void InterruptBA();

    void FillLBAKeyframesWindow(sptam::Map::SharedKeyFrame keyframe);

    // Peform a local bundle adjustment which only adjusts a selection of keyframes.
    bool BundleAdjust(Iterable<sptam::Map::SharedKeyFrame>&& keyFrames);

    void createNewPoints(sptam::Map::SharedKeyFrame& keyFrame);

    /**
     * This tells if refind should try to match a certain point to a certain keyframe.
     * In the Sequential case, since the new_points are from a single keyframe
     * in each iteration, and that keyframe is not checked for refinds,
     * we can assume that the point was not matched before in the given list of keyframes.
     */
    virtual bool isUnmatched(const sptam::Map::KeyFrame& keyFrame, const sptam::Map::SharedPoint& mapPoint);

    virtual void CleanupMap(Iterable<sptam::Map::SharedKeyFrame>&& keyFrames);

    virtual void RemoveMeasurements(Iterable<sptam::Map::SharedMeas>&& measurements);

    void RemoveBadKeyFrames(const ConstIterable<sptam::Map::SharedKeyFrame>& keyFrames);

  // helper functions
  private:

    std::list< sptam::Map::SharedPoint > filterUnmatched(const sptam::Map::KeyFrame& keyFrame, Iterable<sptam::Map::SharedPoint>& mapPoints);

  private:

  /*** Private properties ***/

    BundleDriver bundleDriver_;

    virtual sptam::Map::SharedKeyFrameSet getSafeCovisibleKFs(sptam::Map::SharedKeyFrameSet& baseKFs);

    virtual bool isSafe(sptam::Map::SharedKeyFrame keyframe);


  protected:

  /*** Algorithm thresholds and parameters ***/

    Parameters params_;
};
