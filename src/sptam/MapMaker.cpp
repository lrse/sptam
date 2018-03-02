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

#include "MapMaker.hpp"
#include "BundleDriver.hpp"
#include "match_to_points.hpp"
#include "utils/macros.hpp"
#include "utils/projective_math.hpp"
#include <boost/range/adaptor/indirected.hpp>

#include "opencv2/core/version.hpp"
#if CV_MAJOR_VERSION == 2
  #include <opencv2/highgui/highgui.hpp>
#elif CV_MAJOR_VERSION == 3
  #include <opencv2/highgui.hpp>
#endif

#ifdef SHOW_PROFILING
  #include "utils/log/Profiler.hpp"
  #include "utils/log/ScopedProfiler.hpp"
#endif // SHOW_PROFILING

MapMaker::MapMaker(sptam::Map& map, const Parameters& params)
  : map_( map )
  , rowMatcher_( params.matchingDistanceThreshold, params.descriptorMatcher, params.epipolarDistanceThreshold )
  , params_( params ), keyframe_window_( params.nKeyFramesToAdjustByLocal )
{}

void MapMaker::CleanupMap(ConstIterable<sptam::Map::SharedKeyFrame>&& keyFrames)
{
  #ifdef SHOW_PROFILING
    sptam::ScopedProfiler timer(" ba handleBadPoints: ");
  #endif

  // Find the set of MapPoints watch from the given keyframes (This finding already did it in BA)
  // and select the bad ones
  sptam::Map::SharedMapPointSet bad_points; // use a set to avoid same MapPoints

  for ( const sptam::Map::SharedKeyFrame& keyFrame : keyFrames )
    for ( const sptam::Map::SharedMeas& meas : keyFrame->measurements() )
    {
      const sptam::Map::SharedPoint& mapPoint = meas->mapPoint();

      if ( mapPoint->IsBad() )
        bad_points.insert( mapPoint );
    }

  // Remove the bad MapPoints
  {
    #ifdef SHOW_PROFILING
      sptam::Timer t_remove_bad_points_lock;
      t_remove_bad_points_lock.start();
    #endif

    boost::unique_lock<boost::shared_mutex> lock(map_.map_mutex_);

    #ifdef SHOW_PROFILING
      t_remove_bad_points_lock.stop();
      WriteToLog(" ba remove_bad_points_lock: ", t_remove_bad_points_lock);
    #endif

    for ( const sptam::Map::SharedPoint& mapPoint : bad_points )
      map_.RemoveMapPoint( mapPoint );

    // Remove Keyframes with not enough measurements
    // TODO: Loop Closing doesnt support removing KFs from map yet.
    //~ RemoveBadKeyFrames( keyFrames );
  }
}

sptam::Map::SharedKeyFrame MapMaker::AddKeyFrame(const StereoFrame& frame, /*const */std::list<Match>& measurements)
{
  sptam::Map::SharedKeyFrame keyFrame = map_.AddKeyFrame( frame );

  // Create new 3D points from unmatched features,
  // and save them in the local tracking map.
  createNewPoints( keyFrame );

  // Load matched map point measurements into the new keyframe.
  // the point could have expired in the meantime, so check it.
  for ( auto& match : measurements )
      map_.addMeasurement( keyFrame, match.mapPoint, match.measurement );

  sptam::Map::SharedKeyFrameList new_keyframes;
  new_keyframes.push_back( keyFrame );

  sptam::Map::SharedKeyFrameSet adjustable_keyframes, fixed_keyframes;
  BundleAdjust(new_keyframes, adjustable_keyframes, fixed_keyframes, [](const sptam::Map::SharedKeyFrame&){ return true; });

  return keyFrame;
}

void MapMaker::addStereoPoints(/*const */sptam::Map::SharedKeyFrame& keyFrame, const std::aligned_vector<MapPoint>& points, const std::vector<Measurement>& measurements)
{
  #ifdef SHOW_PROFILING
    sptam::Timer t_lock_add_points;
    t_lock_add_points.start();
  #endif

  boost::unique_lock<boost::shared_mutex> lock(map_.map_mutex_);

  #ifdef SHOW_PROFILING
    t_lock_add_points.stop();
    WriteToLog(" tk lock_add_points: ", t_lock_add_points);
  #endif

  #ifdef SHOW_PROFILING
  {
    sptam::ScopedProfiler timer(" tk add_points: ");
  #endif

  forn ( i, points.size() )
  {
    typename sptam::Map::SharedPoint mapPoint = map_.AddMapPoint( points[ i ] );

    map_.addMeasurement( keyFrame, mapPoint, measurements[ i ] );

    mapPoint->IncreaseMeasurementCount();
  }

  #ifdef SHOW_PROFILING
  }
  #endif
}

void MapMaker::createNewPoints(/*const */sptam::Map::SharedKeyFrame& keyFrame)
{
  std::aligned_vector<MapPoint> points;
  std::vector<Measurement> measurements;

  #ifdef SHOW_PROFILING
  {
    sptam::ScopedProfiler timer(" tk triangulate_points: ");
  #endif

  keyFrame->TriangulatePoints(rowMatcher_, points, measurements);

  #ifdef SHOW_PROFILING
  }
    WriteToLog(" tk created_new_points: ", points.size());
  #endif

  addStereoPoints(keyFrame, points, measurements);
}

bool MapMaker::BundleAdjust(const std::list< sptam::Map::SharedKeyFrame >& new_keyframes, sptam::Map::SharedKeyFrameSet& adjustable_keyframes, sptam::Map::SharedKeyFrameSet& fixed_keyframes, std::function<bool(const sptam::Map::SharedKeyFrame&)> isSafe)
{
  #ifdef SHOW_PROFILING
  sptam::ScopedProfiler timer_totalba(" ba totalba: ");
  #endif

  BundleDriver bundle_adjuster;

  /////////////////
  // Populate BA //
  /////////////////

  keyframe_window_.populateBA(new_keyframes, bundle_adjuster, adjustable_keyframes, fixed_keyframes, isSafe);

  ///////////////////////
  // Refind newly made //
  ///////////////////////

  #ifdef SHOW_PROFILING
  {
    sptam::ScopedProfiler timer(" ba refind_newly_made: ");
  #endif

  std::list<sptam::Map::SharedPoint> new_points = getPointsCreatedBy( new_keyframes );
  /*int nFound = */ReFind( ConstSetIterable<sptam::Map::SharedKeyFrame>::from( adjustable_keyframes ), ConstListIterable<sptam::Map::SharedPoint>::from( new_points ) );

  #ifdef SHOW_PROFILING
  }
  #endif

  //std::cout << "EJECUTANDO BA LOCAL:" << std::endl << std::endl;

  #ifdef SHOW_PROFILING
    sptam::Timer t_adjust;
    t_adjust.start();
  #endif

  bool was_completed = bundle_adjuster.Adjust( params_.maxIterationsLocal );

  #ifdef SHOW_PROFILING
    t_adjust.stop();
    WriteToLog(" ba local_adjust: ", t_adjust);
  #endif

  //std::cout << "DESPUES DE BA LOCAL" << std::endl << std::endl;

  #ifdef SHOW_PROFILING
  {
    sptam::ScopedProfiler timer(" ba local_save_points: ");
  #endif

  // save data even if BA was interrupted
  /* NOTE: Gaston: No need for asking map lock as MapPoints have internal locking */
  bundle_adjuster.SavePoints();

  #ifdef SHOW_PROFILING
  }
  #endif

  #ifdef SHOW_PROFILING
  {
    sptam::ScopedProfiler timer(" ba local_save_cameras: ");
  #endif

  // No lock is required: The keyframes have an internal lock for their modification
  bundle_adjuster.SaveCameras();

  #ifdef SHOW_PROFILING
  }
  #endif

  #ifdef SHOW_PROFILING
  {
    sptam::ScopedProfiler timer(" ba local_handle_bad: ");
  #endif

  if ( was_completed )
  {
    std::list< sptam::Map::SharedMeas > aux = bundle_adjuster.GetBadMeasurements();
    RemoveMeasurements( ListIterable<sptam::Map::SharedMeas>::from( aux ) );
  }
  else
    std::cout << "BA LOCAL NO CONVERGIO" << std::endl;

  #ifdef SHOW_PROFILING
  }
  #endif

  #ifdef USE_LOOPCLOSURE
  /* Notifying newly added keyframe to Loop Closure service */
  if(loopclosing_ != nullptr)
    loopclosing_->addKeyFrames(new_keyframes);
  #endif

  // If Mapper thread was intrrupted, don't do any cleanup.
  if ( not was_completed )
    return was_completed;

  CleanupMap( ConstSetIterable<sptam::Map::SharedKeyFrame>::from( adjustable_keyframes ) );

  return was_completed;
}

std::list<sptam::Map::SharedPoint> MapMaker::getPointsCreatedBy(const sptam::Map::SharedKeyFrame& keyFrame)
{
  std::list<sptam::Map::SharedPoint> newMapPoints;

  for( const auto& measurement : keyFrame->measurements() )
    if ( measurement->GetSource() == Measurement::SRC_TRIANGULATION )
      newMapPoints.push_back( measurement->mapPoint() );

  return newMapPoints;
}

std::list<sptam::Map::SharedPoint> MapMaker::getPointsCreatedBy(const std::list<sptam::Map::SharedKeyFrame>& keyframes)
{
  std::list<sptam::Map::SharedPoint> new_points;

  for ( auto keyframe : keyframes )
  {
    std::list<sptam::Map::SharedPoint> new_points_from_keyframe = getPointsCreatedBy( keyframe );
    new_points.insert(new_points.end(), new_points_from_keyframe.begin(), new_points_from_keyframe.end());
  }

  return new_points;
}

bool MapMaker::isUnmatched(const sptam::Map::KeyFrame& keyFrame, const sptam::Map::SharedPoint& mapPoint)
{
  return not mapPoint->IsBad() and keyFrame.canView( *mapPoint );
}

/**
 * Helper function. Filters points that could be
 * potential matches for a frame.
 */
std::list< sptam::Map::SharedPoint > MapMaker::filterUnmatched(const sptam::Map::KeyFrame& keyFrame, ConstIterable<sptam::Map::SharedPoint>& mapPoints)
{
  std::list< sptam::Map::SharedPoint > filtered_points;

  for ( const sptam::Map::SharedPoint& mapPoint : mapPoints )
    if ( isUnmatched( keyFrame, mapPoint ) )
      filtered_points.push_back( mapPoint );

  return filtered_points;
}

size_t MapMaker::ReFind(ConstIterable<sptam::Map::SharedKeyFrame>&& keyFrames, ConstIterable<sptam::Map::SharedPoint>&& new_points)
{
  // Is there new points?
  if( new_points.empty() )
    return 0;

  size_t nFound = 0;

  for( const sptam::Map::SharedKeyFrame& keyFrame : keyFrames)
  {
    std::list< sptam::Map::SharedPoint > filtered_points = filterUnmatched(*keyFrame, new_points);

    for( const sptam::Map::SharedPoint& mapPoint : filtered_points )
      mapPoint->IncreaseProjectionCount();

    std::list<Match> matches = matchToPoints(
      *keyFrame, ConstListIterable<sptam::Map::SharedPoint>::from( filtered_points )
      , params_.descriptorMatcher, params_.matchingNeighborhoodThreshold
      , params_.matchingDistanceThreshold, Measurement::SRC_REFIND
    );

    for (Match& match : matches) {
      map_.addMeasurement( keyFrame, match.mapPoint, match.measurement );
      match.mapPoint->IncreaseMeasurementCount(); // increase measurement counter of mapPoint
    }

    nFound += matches.size();
  }

  return nFound;
}

void MapMaker::RemoveMeasurements(Iterable<sptam::Map::SharedMeas>&& measurements)
{
  for ( const sptam::Map::SharedMeas& meas : measurements ) {
    meas->mapPoint()->IncreaseOutlierCount();
    map_.removeMeasurement( meas );
  }
}
