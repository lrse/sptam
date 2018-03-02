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

#include "Map.hpp"
#include "tracker_g2o.hpp"
#include "MapMakerThread.hpp"
#include "sptamParameters.hpp"

#include "PosePredictor.hpp"
#include "TrackingReport.hpp"
#include "utils/draw/TrackerView.hpp"

#ifdef USE_LOOPCLOSURE
#include "loopclosing/LoopClosing.hpp"
#include "loopclosing/LCDetector.hpp"
#endif

class SPTAM
{
  public:

    SPTAM(const RowMatcher& rowMatcher, const Parameters& params);

    void init(/*const*/ StereoFrame& frame);

    inline bool isInitialized() const
    { return initialized_; }

    /**
     * Track current frame. If individual images are supplied, they are used to draw results of tracking.
     */
    TrackingReport track(StereoFrame& frame, sptam::TrackerView& tracker_view);

    inline void stop()
    {
      mapMaker_.Stop();
      #ifdef USE_LOOPCLOSURE
      if(loopclosing_)
        loopclosing_->stop();
      #endif
    }

    /* Quick tracking state implementation,
     * this way LoopClosing knows when its safe to correct trayectory */
    bool isTracking();

    #ifdef USE_LOOPCLOSURE
    void setLoopClosing(std::unique_ptr<LCDetector>& loop_detector);
    #endif

    /* Sincronization messages with loopclosure  */
    void setLoopCorrection(const Eigen::Isometry3d& T = Eigen::Isometry3d::Identity());
    void pause();
    void unPause();
    bool isPaused();
    void stopAddingKeyframes();
    void resumeAddingKeyframes();
    bool isAddingKeyframesStopped();

    /* This getters serve as a proxy to the map object for the ros version,
     * this exposes const references to internal lists of the map. 
     * Otherwise the ros node doesnt've access to the map. */
    inline sptam::Map& GetMap()
    { return map_; }

    /* This getters gives unsafe access to the internal referenceKF.
     * This is being used within the lc thread */
    inline sptam::Map::SharedKeyFrame getReferenceKeyframe()
    { return referenceKeyFrame_; }

  protected:

    sptam::Map map_;

    // the map builder interface
    // To run the mapper in a parallel thread, build an instance
    // of MapMakerThread.
    // If the run should be sequential, build an instance of MapMaker.
    #ifdef SINGLE_THREAD
      MapMaker mapMaker_;
    #else
      MapMakerThread mapMaker_;
    #endif

    // the tracker interface
    // To run the the tracker with g2o, build an instance
    // of tracker_g2o.
    // To run with PTAM based code, build an instance
    // of Tracker.
    tracker_g2o tracker_;

    // TODO referenceKeyFrame_ is used by KeyframePolicy function. The KeyframePolicy should be an abstract class instead of a function.
    // The keyframepPolicy should maintain the referenceKeyframe_.
    // Moreover, the KeyframePolicy class should takes as parameter a keyframe policy.
    // referenceKeyframe_ now is used to obtain the local map
    sptam::Map::SharedKeyFrame referenceKeyFrame_;

    /* the set of points from the local map which matched to features in the last frame */
    sptam::Map::SharedMapPointSet tracked_map_;

    #ifdef USE_LOOPCLOSURE
    /* LoopClosing interface created using the loop detector passed. */
    std::shared_ptr<LoopClosing> loopclosing_; // pointer, as may or may-not be created
    #endif

    // This mutex must be use to protect isPause, isTracking, isAddingKeyframesStopped booleans and the error loop correction lc_T matrix.
    mutable std::mutex pause_mutex_;
    bool isTracking_ = false;
    bool isPaused_ = false;
    bool isAddingKeyframesStopped_ = false;
    void setTracking(bool);

    /* In case of a loop correction, the lc thread will report the acumulated error perceived
     * otherwise the Mat will be empty. This will serve as a correction for the posepredictor/motionmodel
     * prior received. The error its expresed as the delta tranformation between the two loop keyframes */
    Eigen::Isometry3d lc_T_ = Eigen::Isometry3d::Identity(); // 4x4 dimension!

    Parameters params_;

    RowMatcher rowMatcher_;

  // some system statistics

    size_t frames_since_last_kf_;

  // helper functions

    /**
     * @brief select the map points that are relevant to a frame (i.e. the local map).
     *   Map points are filtered by frustum culling and by the
     *   angle-of-view of the last descriptor.
     *   TODO: This could be asked to the map and the map should returns it in an efficient way
     */
    void filterPoints(const StereoFrame& frame, sptam::Map::SharedMapPointList &localMap, sptam::Map::SharedKeyFrameSet &localKeyFrames);

    bool shouldBeKeyframe(const StereoFrame& frame, const std::list<Match>& measurements);

  private:

    bool initialized_;

  public:

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
