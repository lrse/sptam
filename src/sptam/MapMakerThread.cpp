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

#include "MapMakerThread.hpp"
#include "utils/Profiler.hpp"
#include "utils/Logger.hpp"
#include "utils/macros.hpp"

MapMakerThread::MapMakerThread(
  sptam::Map& map,
  const CameraParameters& cameraCalibrationLeft,
  const CameraParameters& cameraCalibrationRight,
  const double stereo_baseline,
  const Parameters& params
)
  : MapMaker( map, cameraCalibrationLeft, cameraCalibrationRight, stereo_baseline, params )
  , globalBAIsRunning_(false), stop_(false)
  , maintenanceThread_(&MapMakerThread::Maintenance, this)
{}

void MapMakerThread::AddKeyFrame(StereoFrame::UniquePtr& keyFrame)
{
  // Tell the mapmaker to stop doing low-priority stuff
  // and concentrate on this KF first.
  skipMaintenanceIteration_ = true;

  // break the bundle adjustment if its running
  if ( globalBAIsRunning_ ) {
    std::cout << "-- -- -- INTERRUPT GLOBAL BA -- -- --" << std::endl;
    InterruptBA();
    std::cout << "-- -- -- LISTO!!! -- -- --" << std::endl;
  }

  // push new keyframe to queue and signal
  keyFrameQueue_.push( keyFrame.release() );

  // TODO ojo que acá podría suceder que el mapper haga un push,
  // ya que esta funcion se ejecuta en el hilo del tracker.
  // En este caso, la llamada de abajo podría tener uno (o más)
  // frames de más.

  #ifdef SHOW_PROFILING
    WriteToLog(" tk queueSize: ", keyFrameQueue_.size());
  #endif
}

void MapMakerThread::Stop()
{
  // Wait until Mapper empty the queue
  std::cout << "-- -- -- WAIT QUEUE - -- --" << std::endl;
  keyFrameQueue_.waitEmpty();

  stop_ = true;
  skipMaintenanceIteration_ = true;

  InterruptBA();

  keyFrameQueue_.stop();

  std::cout << "-- -- -- WAIT JOIN -- -- --" << std::endl;
  maintenanceThread_.join();
}

void MapMakerThread::Maintenance()
{
  while ( not stop_ ) {

    skipMaintenanceIteration_ = false;

    std::list<StereoFrame*> keyframes;

    StereoFrame* keyframe;
    bool moreData = keyFrameQueue_.waitAndPop( keyframe );
    keyframes.push_back( keyframe );

    while ( (keyframes.size() < 5) and moreData )
    {
      moreData = keyFrameQueue_.waitAndPop( keyframe );
      keyframes.push_back( keyframe );
    }

    // An abrupt stop is necessary because the data aquired by
    // waitAndPop() in case of a shutdown may be garbage.
    if ( stop_ )
      return;

    #ifdef SHOW_PROFILING
      double start_total = GetSeg();
    #endif

    // TODO hace falta? me parece que son muy poco probables los casos
    // en los que serviría de algo como para andarlo corriendo todo el tiempo
    // It is necessasry to remove bad measurements.
    // Just in case, some point was erased after it was observed
    // HandleBadPoints(); // si esta funcion esta comentada puede ser que se rompa sptam?

    #ifdef SHOW_PROFILING
      double start_refind = GetSeg();
    #endif

    // esta funcion tiene un costo computacional muy alto (mejora la precision si se la utiliza)
    // obtener mas mediciones para los KeyFrames que se quiere agregar al mapa
//    for ( auto keyFrame : keyframes ) {
//      int nFoundNow = ReFindInSingleKeyFrame( *keyFrame );
//      std::cout << "Refined Points: " << nFoundNow << std::endl;
//    }

    #ifdef SHOW_PROFILING
      double end_refind = GetSeg();
      WriteToLog(" ba ReFindInSingleKeyFrame: ", start_refind, end_refind);
    #endif

    // Add KeyFrames to the Map
    for ( auto keyFrame : keyframes ) {
      map_.AddKeyFrame( keyFrame );
    }

    // Get New MapPoints triangulated by the keyframes
    for ( auto keyFrame : keyframes ) {
      const std::vector<MapPoint*>& newKeyFramePoints = GetNewMapPoints( *keyFrame );
      newPoints_.reserve( newPoints_.size() + newKeyFramePoints.size() );
      newPoints_.insert( newPoints_.end(), newKeyFramePoints.begin(), newKeyFramePoints.end() );
    }

    #ifdef SHOW_PROFILING
      std::cout << "Se agrega/n " << keyframes.size() << " KeyFrame al mapa." << std::endl;

      WriteToLog(" ba totalKeyFrames: ", map_.nKeyFrames());
      WriteToLog(" ba totalPoints: ", map_.nMapPoints());
    #endif

    // general maintenance

    // Refine Newly made points (the ones added from stereo matches
    // when the last keyframe came in)

    #ifdef SHOW_PROFILING
      double start_refine = GetSeg();
    #endif

    // First, make a list of the n keyframes we want adjusted in the adjuster.
    // This will be the lasts keyframe inserted
    std::vector<StereoFrame*> keyframesRefined = LastKeyFrames( nKeyFramesToAdjustByLocal_ );

    /*int nFound = */ReFindNewlyMade( keyframesRefined );

    #ifdef SHOW_PROFILING
      double end_refine = GetSeg();
      WriteToLog(" ba ReFindNewlyMade: ", start_refine, end_refine);
    #endif

    // LOCAL Bundle Adjustment
    // LOCAL BA must run always on constrast to original PTAM
    // because Not necessarily GLOBAL BA will run because it could be interrupted

    #ifdef SHOW_PROFILING
      double start_local = GetSeg();
    #endif

    BundleAdjustRecent();

    #ifdef SHOW_PROFILING
      double end_local = GetSeg();
      WriteToLog(" ba local: ", start_local, end_local);
    #endif

    // Mapper thread was intrrupted?
    if ( skipMaintenanceIteration_ )
      continue;

    // GLOBAL Bundle Adjustment

    #ifdef SHOW_PROFILING
      double start_global = GetSeg();
    #endif

    // set Global BA is running
    globalBAIsRunning_ = true;

//    BundleAdjustAll();

    // set Global BA is not running
    globalBAIsRunning_ = false;

    #ifdef SHOW_PROFILING
      double end_global = GetSeg();
      WriteToLog(" ba global: ", start_global, end_global);
    #endif

    // Remove bad points marked by BA

    CleanupMap();

    #ifdef SHOW_PROFILING
      double end_total = GetSeg();
      WriteToLog(" ba totalba: ", start_total, end_total);
    #endif
  }
}
