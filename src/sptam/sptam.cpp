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

#include "sptam.hpp"
#include "Match.hpp"
#include "match_to_points.hpp"
#include "utils/macros.hpp"
#include <opencv2/core/eigen.hpp>
#include "utils/projective_math.hpp"
#include "utils/eigen_alignment.hpp"

#include "KeyFramePolicy.hpp"
#include "FeatureExtractorThread.hpp"
#include <boost/range/adaptor/indirected.hpp>

#include <boost/thread/locks.hpp>
#include <boost/thread/lock_types.hpp>

#ifdef ENABLE_PARALLEL_CODE
#include <tbb/parallel_for.h>
#endif

#ifdef SHOW_PROFILING
  #include "../sptam/utils/log/Profiler.hpp"
#endif // SHOW_PROFILING

// ===================== //
// Some magic numbers :) //
// ===================== //

// Minimum number of triangulated points required to initialize a "good" map.
#define MIN_POINTS_FOR_MAP 10

SPTAM::SPTAM(const RowMatcher& rowMatcher, const Parameters& params)
  : mapMaker_(map_, params)
  , referenceKeyFrame_( nullptr )
  , params_( params )
  , rowMatcher_( rowMatcher )
  , frames_since_last_kf_( 0 )
  , initialized_( false )
{
#ifdef OPENCV_THREADS
  cv::setNumThreads(OPENCV_THREADS);
  std::cout << "OpenCV: threads set to " << OPENCV_THREADS << std::endl;
#endif
}

void SPTAM::init(/*const*/ StereoFrame& frame)
{
  std::aligned_vector<MapPoint> points;
  std::vector<Measurement> measurements;

  frame.TriangulatePoints(rowMatcher_, points, measurements);

  // check that there are at least a minimum number of correct matches when there is no map
  if ( points.size() < MIN_POINTS_FOR_MAP )
    throw std::runtime_error("Not enough points to initialize map.");

  // Add Keyframe to the map
  sptam::Map::SharedKeyFrame keyFrame = map_.AddKeyFrame( frame );

  mapMaker_.addStereoPoints(keyFrame, points, measurements);

  referenceKeyFrame_ = keyFrame;

  initialized_ = true;

  // add new points to the tracked_map
  for(auto& meas : referenceKeyFrame_->measurements()) {
    sptam::Map::SharedPoint sharedPoint = meas->mapPoint();
    tracked_map_.insert( sharedPoint );
  }
}

TrackingReport SPTAM::track(StereoFrame& frame, sptam::TrackerView& tracker_view)
{
  while(isPaused())
    std::this_thread::yield();

  setTracking(true);

  frames_since_last_kf_++;

  CameraPose estimatedCameraPose = frame.GetCameraPose();

  TrackingReport report;

  #ifdef USE_LOOPCLOSURE
  {
    std::lock_guard<std::mutex> lock(pause_mutex_);
    if(not lc_T_.matrix().isIdentity()){ // The lc thread its updating a recent part of the map, we need to wait until finish
      Eigen::Isometry3d estimatedPose; // CameraPose to Isometry pose matrix
      estimatedPose.linear() = estimatedCameraPose.GetOrientationMatrix();
      estimatedPose.translation() = estimatedCameraPose.GetPosition();

      estimatedPose = estimatedPose * lc_T_; // applying loop correction

      /* re-setting estimated camera pose */
      Eigen::Quaterniond orientation(estimatedPose.linear());
      estimatedCameraPose = CameraPose(estimatedPose.translation(), orientation, Eigen::Matrix6d::Identity());
      frame.UpdateCameraPose(estimatedCameraPose);

      report.T_corr = lc_T_.matrix();

      lc_T_.setIdentity();
    }
  }
  #endif

  filterPoints( frame, report.localMap, report.localKeyFrames );

  // Try to match the features found in the new frame to the
  // 3D points saved in the map.
  #ifdef SHOW_PROFILING
    sptam::Timer t_find_matches;
    t_find_matches.start();
  #endif

  // Here we will save the measured features in this frame,
  // to be passed to the tracker for pose refinement.
  std::list<Match> measurements = matchToPoints(
    frame, ConstListIterable<sptam::Map::SharedPoint>::from( report.localMap ),
    params_.descriptorMatcher, params_.matchingNeighborhoodThreshold,
    params_.matchingDistanceThreshold, Measurement::SRC_TRACKER
  );

  #ifdef SHOW_PROFILING
    t_find_matches.stop();
    WriteToLog(" tk find_matches: ", t_find_matches);
    WriteToLog( " tk matched_feat_total: ", measurements.size() );
  #endif

  #ifdef SHOW_PROFILING
    sptam::Timer t_update_descriptors;
    t_update_descriptors.start();
  #endif

  tracked_map_.clear();

  for ( auto& match : measurements )
  {
    // Update the descriptor of the MapPoint with the new one.
    // It is not needed to ask a lock to the map to modify the descriptor of the mappoint (they have internal locking)
    match.mapPoint->updateDescriptor( match.measurement.GetDescriptor() );

    match.mapPoint->IncreaseMeasurementCount();

    tracked_map_.insert(match.mapPoint);
  }

  #ifdef SHOW_PROFILING
    t_update_descriptors.stop();
    WriteToLog(" tk update_descriptors: ", t_update_descriptors);
  #endif

#ifdef SHOW_PROFILING
  sptam::Timer t_draw;
#endif

  #ifdef SHOW_TRACKED_FRAMES

#ifdef SHOW_PROFILING
  t_draw.start();
#endif

  tracker_view.draw(frame, report.localMap, measurements, params_, true);

#ifdef SHOW_PROFILING
  t_draw.stop();
#endif

  #endif // SHOW_TRACKED_FRAMES

  #ifdef SHOW_PROFILING
    sptam::Timer t_tracking_refine;
    t_tracking_refine.start();
  #endif

  // The tracker will try to refine the predicted camera pose
  // by reducing the reprojection error from the matched features.

  try {

    report.refinedCameraPose = tracker_.RefineCameraPose(
      estimatedCameraPose, frame.getRectifiedCameraParameters(), measurements
    );

    #ifdef SHOW_PROFILING
      t_tracking_refine.stop();
      WriteToLog(" tk tracking_refine: ", t_tracking_refine);
    #endif

    frame.UpdateCameraPose( report.refinedCameraPose );


    report.trackedMap = tracked_map_;

    report.state = TrackingReport::State::OK;

  } catch( tracker_g2o::not_enough_points& err ) {

    report.refinedCameraPose = estimatedCameraPose;

    report.state = TrackingReport::State::NOT_ENOUGH_POINTS;

    #ifdef SHOW_PROFILING
      STREAM_TO_LOG( frame.GetId() << " WARNING: " << err.what() << ". measurements: " << measurements.size() );
    #endif // SHOW_PROFILING
  }

  #ifdef SHOW_TRACKED_FRAMES
  #ifdef SHOW_PROFILING
  t_draw.start();
  #endif
  tracker_view.draw(frame, report.localMap, measurements, params_, false);
  #ifdef SHOW_PROFILING
  t_draw.stop();
  #endif
  #endif // SHOW_TRACKED_FRAMES

  if( report.isOk() and shouldBeKeyframe( frame, measurements ) )
  {
    #ifdef SHOW_PROFILING
      sptam::Timer t_add_keyframe;
      t_add_keyframe.start();
    #endif

    // The mapMaker takes ownership of the UniquePtr,
    // so after this call the variable should not be used.
    sptam::Map::SharedKeyFrame keyFrame = mapMaker_.AddKeyFrame( frame, measurements );

    #ifdef SHOW_PROFILING
      t_add_keyframe.stop();
      WriteToLog(" tk add_keyframe: ", t_add_keyframe);
    #endif

    referenceKeyFrame_ = keyFrame;
    frames_since_last_kf_ = 0;

    // add new points to the tracked_map_
    for(auto& meas : referenceKeyFrame_->measurements()) {
      sptam::Map::SharedPoint sharedPoint = meas->mapPoint();
      tracked_map_.insert( sharedPoint );

      cv::Vec3b color = tracker_view.featureColor( *meas );
      sharedPoint->setColor(color);
    }

    //std::cout << "added keyframe " << (uintptr_t)lastKeyFrame_.get() << " with " << lastKeyFrame_->measurements().size() << std::endl;
  }
  //else std::cout << "no new keyframe" << std::endl;

  /* Quick tracking state implementation
   * this way LoopClosing knows when its safe to correct trayectory */
  setTracking(false);

#ifdef SHOW_PROFILING
  WriteToLog(" tk drawFrames: ", t_draw);
#endif

  return report;
}

void SPTAM::filterPoints(const StereoFrame& frame, sptam::Map::SharedMapPointList& localMap, sptam::Map::SharedKeyFrameSet& localKeyFrames)
{
  sptam::Map::SharedMapPointSet localMapSet;

  {
    #ifdef SHOW_PROFILING
    sptam::Timer t_lock_frustum;
    t_lock_frustum.start();
    #endif

    /* Lock for reading a small segment of the Map, after getting the required points locking is no longer
       required due to internal locking of MapPoints */
    boost::shared_lock<boost::shared_mutex> lock(map_.map_mutex_);

    #ifdef SHOW_PROFILING
    t_lock_frustum.stop();
    WriteToLog(" tk lock_frustum: ", t_lock_frustum);
    #endif

    #ifdef SHOW_PROFILING
    sptam::Timer t_localmap;
    t_localmap.start();
    #endif

    map_.getLocalMap( tracked_map_, localMapSet, localKeyFrames, referenceKeyFrame_ );

    #ifdef SHOW_PROFILING
    t_localmap.stop();
    WriteToLog(" tk getLocalMap: ", t_localmap);
    WriteToLog(" tk localPoints: ", localMapSet.size());
    #endif
  }

#ifdef SHOW_PROFILING
  sptam::Timer t_frustum;
  t_frustum.start();
#endif

#ifdef ENABLE_PARALLEL_CODE
  std::vector<sptam::Map::SharedPoint> localMapVector;
  localMapVector.reserve(localMapSet.size());
  std::copy(localMapSet.begin(), localMapSet.end(), std::back_inserter(localMapVector));

  std::vector<bool> keptPoints(localMapVector.size(), false);

  tbb::parallel_for(tbb::blocked_range<size_t>(0, localMapVector.size()),
    [&](const tbb::blocked_range<size_t>& range) {
      for (size_t i = range.begin(); i != range.end(); i++)
      {
        const sptam::Map::SharedPoint& mapPoint = localMapVector[i];
        if ( not mapPoint->IsBad() and frame.canView( *mapPoint ) )
        {
          mapPoint->IncreaseProjectionCount();
          keptPoints[i] = true;
        }
      }
    }, tbb::auto_partitioner() );

  for (size_t i = 0; i < localMapVector.size(); i++)
    if (keptPoints[i]) localMap.push_back(localMapVector[i]);

#else
  for ( const sptam::Map::SharedPoint& mapPoint : localMapSet )
    if ( not mapPoint->IsBad() and frame.canView( *mapPoint ) )
    {
      mapPoint->IncreaseProjectionCount();
      localMap.push_back( mapPoint );
    }
#endif

  #ifdef SHOW_PROFILING
    t_frustum.stop();
    WriteToLog(" tk frustum: ", t_frustum);
    WriteToLog(" tk visiblePoints: ", localMap.size());
  #endif
}

bool SPTAM::shouldBeKeyframe(const StereoFrame& frame, const std::list<Match> &measurements)
{

  /* Check if adding keyframes is permitted */
  if(isAddingKeyframesStopped()){
    #ifdef SHOW_PROFILING
    WriteToLog(" tk AddingKframes is paused: ", 1);
    #endif
    return false;
  }

  #ifdef SHOW_PROFILING
  sptam::Timer t_keyframe_selection;
  t_keyframe_selection.start();
  #endif

  // the Heuristicas off keyframe selection are a trade-off between performance and accuacy
  // more keyframes are selected, more keyframes will be queue in mapping thread, and the precision of the system will be compromised

  // Based free coverage image Policy
//  return AddKeyFrameImageCoverPolicy(frame, 1241, 0.2);

  // Based Matched Features percentage Policy
//  return AddKeyFrameFeaturesPolicy(frame, 0.1);


  // Based Tracking features percentage wrt to reference keyframe Policy
  size_t numStereoMeas = measurements.size();
  bool shouldBeKeyframe = (AddKeyFrameTrackingFeaturesPolicy(frame, measurements, *referenceKeyFrame_, params_.minimumTrackedPointsRatio)
                          or (numStereoMeas < 20)); // number of matches less than a threshold

  //std::cout << "stereo meas: " << numStereoMeas << std::endl;

  #ifdef SHOW_PROFILING
  t_keyframe_selection.stop();
  WriteToLog(" tk keyframe_selection_strategy: ", t_keyframe_selection);
  #endif

  return shouldBeKeyframe;
}

#ifdef USE_LOOPCLOSURE
void SPTAM::setLoopClosing(std::unique_ptr<LCDetector>& loop_detector)
{
  if(loop_detector)
  {
    loopclosing_.reset(new LoopClosing(*this, mapMaker_, map_, loop_detector, {params_.descriptorMatcher, params_.matchingDistanceThreshold}));
    mapMaker_.setLoopClosing(loopclosing_);
   }
}
#endif

void SPTAM::setLoopCorrection(const Eigen::Isometry3d& T)
{
  std::lock_guard<std::mutex> lock(pause_mutex_);
  lc_T_ = T; // 4x4 copy
}

/* NOTE: Pausing needs to be not blocking, because the tracker isnt actually a diferent thread. */
void SPTAM::pause(){
  std::lock_guard<std::mutex> lock(pause_mutex_);
  isPaused_ = true;
}

void SPTAM::unPause(){
  std::lock_guard<std::mutex> lock(pause_mutex_);
  isPaused_ = false;
}

bool SPTAM::isPaused(){
  std::lock_guard<std::mutex> lock(pause_mutex_);
  return isPaused_;
}

bool SPTAM::isTracking(){
  std::lock_guard<std::mutex> lock(pause_mutex_);
  return isTracking_;
}

void SPTAM::setTracking(bool set){
  std::lock_guard<std::mutex> lock(pause_mutex_);
  isTracking_ = set;
}

void SPTAM::stopAddingKeyframes()
{
  std::lock_guard<std::mutex> lock(pause_mutex_);
  isAddingKeyframesStopped_ = true;
}

void SPTAM::resumeAddingKeyframes()
{
  std::lock_guard<std::mutex> lock(pause_mutex_);
  isAddingKeyframesStopped_ = false;
}

bool SPTAM::isAddingKeyframesStopped()
{
  std::lock_guard<std::mutex> lock(pause_mutex_);
  return isAddingKeyframesStopped_;
}
