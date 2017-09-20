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

#include <algorithm>
#include "MapMakerThread.hpp"
#include "utils/macros.hpp"

#ifdef SHOW_PROFILING
#include "../sptam/utils/log/Profiler.hpp"
#include "../sptam/utils/log/ScopedProfiler.hpp"
#endif

MapMakerThread::MapMakerThread(sptam::Map& map, const Parameters& params)
  : MapMaker( map, params ), stop_( false ), processing_(false)
{
  maintenanceThread_ = std::thread(&MapMakerThread::Maintenance, this);
}

sptam::Map::SharedKeyFrame MapMakerThread::AddKeyFrame(const StereoFrame& frame, /*const */std::list<Match>& measurements)
{

  sptam::Map::SharedKeyFrame keyFrame;
  {
    boost::unique_lock<boost::shared_mutex> scoped_lock( map_.map_mutex_ );
    keyFrame = map_.AddKeyFrame( frame );
  }

  // Create new 3D points from unmatched features,
  // and save them in the local tracking map.
  createNewPoints( keyFrame );

  // Load matched map point measurements into the new keyframe.
  {
    #ifdef SHOW_PROFILING
      sptam::Timer t_map_lock;
      t_map_lock.start();
    #endif

    boost::unique_lock<boost::shared_mutex> scoped_lock( map_.map_mutex_ );

    #ifdef SHOW_PROFILING
      t_map_lock.stop();
      WriteToLog(" tk lock_add_meas: ", t_map_lock);
    #endif

    #ifdef SHOW_PROFILING
    {
      sptam::ScopedProfiler timer(" tk add_meas: ");
    #endif

    // the point could have expired in the meantime, so check it.
    for ( auto& match : measurements )
      map_.addMeasurement( keyFrame, match.mapPoint, match.measurement );

    #ifdef SHOW_PROFILING
    }
    #endif
  }

  /*#ifdef SHOW_PROFILING
    WriteToLog(" ba totalKeyFrames: ", map_.nKeyFrames());
    WriteToLog(" ba totalPoints: ", map_.nMapPoints());
  #endif*/

  #ifdef DISABLE_LOCALMAPPING
  #ifdef USE_LOOPCLOSURE
  /* Local Mapping is disabled, pass new keyframes to LC inmediately */
  if(loopclosing_ != nullptr)
    loopclosing_->addKeyFrame(keyFrame);
  #endif
  #else // DISABLE_LOCALMAPPING
  // push new keyFrame to queue and signal
  keyFrameQueue_.push( keyFrame );
  requests_[PROCESS_REQUEST] = true;
  requests_cv_.notify_all();
  #endif

  // WARNING ojo que acá podría suceder que el mapper haga un push,
  // ya que esta funcion se ejecuta en el hilo del tracker.
  // En este caso, la llamada de abajo podría tener uno (o más)
  // frames de más.

  #ifdef SHOW_PROFILING
    WriteToLog(" tk queueSize: ", keyFrameQueue_.size());
  #endif

  return keyFrame;
}

void MapMakerThread::Stop()
{
  // Wait until Mapper empty the queue
  std::cout << "-- -- -- WAIT QUEUE - -- --" << std::endl;
  keyFrameQueue_.waitEmpty();

  stop_ = true;

  keyFrameQueue_.stop();

  requests_cv_.notify_all();

  std::cout << "-- -- -- WAIT JOIN -- -- --" << std::endl;
  maintenanceThread_.join();
}

void MapMakerThread::Maintenance()
{
  while ( not stop_ )
  {

    processing_ = false;

    std::bitset<2> requests;

    {
      std::unique_lock<std::mutex> lock(requests_mutex_);

      requests_cv_.wait(lock, [this]{return stop_ or requests_.any();});

      requests = requests_; // Copy request to local variable
      requests_.reset(); // Clearing requests
    }

    if ( stop_ ) return;

    processing_ = true;

    if(requests[LOCKWINDOW_REQUEST])
      attendLockUsageWindow();

    if(requests[PROCESS_REQUEST] and (not keyFrameQueue_.empty()))
    {
      attendNewKeyframes();

      if(not keyFrameQueue_.empty())
      {
        std::unique_lock<std::mutex> lock(requests_mutex_);
        requests_[PROCESS_REQUEST] = true;
      }
    }
  }
}

void MapMakerThread::attendLockUsageWindow()
{
  /* This is a safe portion of code where to define safe window*/
  std::lock_guard<std::mutex> lock(usagewindow_mutex_);

  for (sptam::Map::SharedKeyFrame& keyFrame : LBA_keyframes_window_){
    locked_window_.insert( keyFrame );
    for( auto it : keyFrame->covisibilityKeyFrames() )
      locked_window_.insert( it.first );
  }

  isUsageWindowLocked_ = true;
}

void MapMakerThread::attendNewKeyframes()
{
  // Select at most 5 new unprocessed keyFrames to refine.
  std::list< sptam::Map::SharedKeyFrame > keyFrames;
  {
    sptam::Map::SharedKeyFrame keyFrame;
    bool moreData = keyFrameQueue_.waitAndPop( keyFrame );
    keyFrames.push_back( keyFrame );

    while ( (keyFrames.size() < 5) and moreData )
    {
      moreData = keyFrameQueue_.waitAndPop( keyFrame );
      keyFrames.push_back( keyFrame );
    }
  }

  #ifdef SHOW_PROFILING
    sptam::ScopedProfiler timer(" ba totalba: ");
  #endif

  // Get New MapPoints triangulated by the keyFrames
  std::list<sptam::Map::SharedPoint> new_points;

  // Fill in new_points and update LBA_keyframes_window_.
  LBA_keyframes_window_.clear(); // clear previous Local BA processed keyframes
  for ( auto keyFrame : keyFrames )
  {
    std::list<sptam::Map::SharedPoint> newKeyFramePoints = getPointsCreatedBy( keyFrame );

    new_points.insert(new_points.end(), newKeyFramePoints.begin(), newKeyFramePoints.end());

    // update cache buffer
    LBA_keyframes_window_.push_back( keyFrame );
  }

  // most recent keyframe extracted from the queue
  sptam::Map::SharedKeyFrame recentKeyframe = keyFrames.back();

  FillLBAKeyframesWindow( recentKeyframe );

  /*#ifdef SHOW_PROFILING
    //std::cout << "Se agrega/n " << keyframes.size() << " KeyFrame al mapa." << std::endl;

    WriteToLog(" ba totalKeyFrames: ", map_.nKeyFrames());
    WriteToLog(" ba totalPoints: ", map_.nMapPoints());
  #endif*/

  // general maintenance

  // Refine Newly made points (the ones added from stereo matches
  // when the last keyframe came in)

  #ifdef SHOW_PROFILING
  {
    sptam::ScopedProfiler timer(" ba refind_newly_made: ");
  #endif

  /*int nFound = */ReFind( ListIterable<sptam::Map::SharedKeyFrame>::from( LBA_keyframes_window_ ), ListIterable<sptam::Map::SharedPoint>::from( new_points ) );

  #ifdef SHOW_PROFILING
  }
  #endif

  #ifdef SHOW_PROFILING
    sptam::Timer t_local;
    t_local.start();
  #endif

  // No lock is required: LBA_keyframes_window_ is an internal variable and it is not modified by other thread
  // No lock is required: The keyframes have an internal lock for their modification
  bool completed = BundleAdjust( ListIterable<sptam::Map::SharedKeyFrame>::from( LBA_keyframes_window_ ) );

  #ifdef SHOW_PROFILING
    t_local.stop();
    WriteToLog(" ba local: ", t_local);
  #endif

  #ifdef USE_LOOPCLOSURE
  /* Notifying newly added keyframe to Loop Closure service */
  if(loopclosing_ != nullptr)
    loopclosing_->addKeyFrames(keyFrames);
  #endif

  // If Mapper thread was intrrupted, don't do any cleanup.
  if ( not completed )
    return;

  // Remove bad points marked by BA
  CleanupMap( ListIterable<sptam::Map::SharedKeyFrame>::from( LBA_keyframes_window_ ) );
}

// Check if the local window used by the LBA is locked
bool MapMakerThread::isSafe(sptam::Map::SharedKeyFrame keyframe) {
  if( isUsageWindowLocked() )
    return (locked_window_.count( keyframe ) == 1);
  else
    return true;
}


void MapMakerThread::waitUntilEmptyQueue()
{ keyFrameQueue_.waitEmpty(); }

bool MapMakerThread::isProcessing()
{ return processing_.load(); }

const sptam::Map::SharedKeyFrameSet& MapMakerThread::lockUsageWindow()
{
  {
    // Requesting the establishment of a safe window
    std::lock_guard<std::mutex> lock(usagewindow_mutex_);

    isUsageWindowLocked_ = false;
    locked_window_.clear();

    requests_[LOCKWINDOW_REQUEST] = true;
    requests_cv_.notify_all();
  }

  while(!isUsageWindowLocked())
    std::this_thread::yield();

  return locked_window_;
}

void MapMakerThread::freeUsageWindow()
{
  std::lock_guard<std::mutex> lock(usagewindow_mutex_);

  isUsageWindowLocked_ = false;
  locked_window_.clear();
}

bool MapMakerThread::isUsageWindowLocked()
{
  std::lock_guard<std::mutex> lock(usagewindow_mutex_);
  return isUsageWindowLocked_;
}

bool hasMeasurement(const sptam::Map::KeyFrame& keyFrame, const sptam::Map::SharedPoint& mapPoint)
{
  for (const auto& meas : keyFrame.measurements() )
    if (meas->mapPoint().get() == mapPoint.get())
      return true;

  return false;
}

bool MapMakerThread::isUnmatched(const sptam::Map::KeyFrame& keyFrame, const sptam::Map::SharedPoint& mapPoint)
{
  return not hasMeasurement(keyFrame, mapPoint) and MapMaker::isUnmatched(keyFrame, mapPoint);
}
