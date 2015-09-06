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

#include "MapMaker.hpp"
#include "utils/macros.hpp"
#include "utils/projective_math.hpp"
#include <opencv2/highgui/highgui.hpp>

#ifdef SHOW_PROFILING
  #include "utils/Profiler.hpp"
  #include "utils/Logger.hpp"
#endif // SHOW_PROFILING

MapMaker::MapMaker(
  sptam::Map& map,
  const CameraParameters& cameraCalibrationLeft,
  const CameraParameters& cameraCalibrationRight,
  const double stereo_baseline,
  const Parameters& params
)
  : map_(map)
  , bundleDriver_(cameraCalibrationLeft.intrinsic, cameraCalibrationRight.intrinsic, stereo_baseline)
  , cameraCalibrationLeft_(cameraCalibrationLeft), cameraCalibrationRight_(cameraCalibrationRight)
  , descriptorMatcher_(params.descriptorMatcher)
  , nKeyFramesToAdjustByLocal_( 10 )
  , maxIterationsGlobal_( 20 )
  , maxIterationsLocal_( 20 )
  , matchingDistanceThreshold_( params.matchingDistanceThreshold )
  , keyFrameDistanceThreshold_( params.keyFrameDistanceThreshold )
  , matchingNeighborhood_( params.matchingNeighborhoodThreshold )
{}

void MapMaker::InterruptBA()
{
  bundleDriver_.Break();
}

void MapMaker::CleanupMap()
{
  #ifdef SHOW_PROFILING
    double start_handle = GetSeg();
  #endif

  map_.RemoveBadPoints();

  map_.RemoveBadKeyFrames();

  #ifdef SHOW_PROFILING
    double end_handle = GetSeg();
    WriteToLog(" ba handleBadPoints: ", start_handle, end_handle);
  #endif
}

void MapMaker::AddKeyFrame(StereoFrame::UniquePtr& keyFramePtr)
{

  #ifdef SHOW_PROFILING
    double start_total = GetSeg();
  #endif

  StereoFrame* keyFrame = keyFramePtr.release();

  map_.AddKeyFrame( keyFrame );

  #ifdef SHOW_PROFILING
    std::cout << "Se agrega un KeyFrame al mapa." << std::endl;

    WriteToLog(" ba totalKeyFrames: ", map_.nKeyFrames());
    WriteToLog(" ba totalPoints: ", map_.nMapPoints());
  #endif

  #ifdef SHOW_PROFILING
    double start_refind = GetSeg();
  #endif

  // esta funcion tiene un costo computacional muy alto (mejora la precision si se la utiliza)
  // obtener mas mediciones para los KeyFrames que se quiere agregar al mapa
//  int nFoundNow = ReFindInSingleKeyFrame( *keyFrame );
//  std::cout << "Refined Points: " << nFoundNow << std::endl;

  #ifdef SHOW_PROFILING
    double end_refind = GetSeg();
    WriteToLog(" ba ReFindInSingleKeyFrame: ", start_refind, end_refind);
  #endif

  // GENERAL MAINTAINANCE


  // Refine Newly made points (the ones added from stereo matches
  // when the last keyframe came in)

  // Get New MapPoints triangulated by the keyframes
  newPoints_ = GetNewMapPoints( *keyFrame );

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

  // Global Bundle Adjustment

  #ifdef SHOW_PROFILING
    double start_global = GetSeg();
  #endif


//  BundleAdjustAll();

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

bool MapMaker::NeedNewKeyFrame(const StereoFrame& kCurrent)
{
  StereoFrame* pClosest = ClosestKeyFrame(kCurrent);
  double dDist = KeyFrameLinearDist(kCurrent, *pClosest);

  if(dDist > keyFrameDistanceThreshold_)
    return true;

  return false;
}

const std::vector<StereoFrame*> MapMaker::LastKeyFrames(const unsigned int n) const
{
  std::vector<StereoFrame*> lastKeyFrames = ( map_.nKeyFrames() < n ) ?
      map_.GetKeyFrames() : std::vector<StereoFrame*>(map_.GetKeyFrames().end() - n, map_.GetKeyFrames().end());

  return lastKeyFrames;
}

double MapMaker::KeyFrameLinearDist(const StereoFrame& k1, const StereoFrame& k2)
{
  // TODO deberia ser GetPosition()
  cv::Point3d v3Diff = k1.GetPosition() - k2.GetPosition();

  return sqrt( v3Diff.dot( v3Diff ) );
}

std::vector<StereoFrame*> MapMaker::NClosestKeyFrames(StereoFrame& k, unsigned int N)
{
  std::vector<std::pair<double, StereoFrame* > > vKFandScores;
  for( auto& keyFrame : map_.GetKeyFrames() ) {
    if(keyFrame == &k)
      continue;
    double dDist = KeyFrameLinearDist(k, *keyFrame);
    vKFandScores.push_back(std::make_pair(dDist, keyFrame));
  }
  if(N > vKFandScores.size())
    N = vKFandScores.size();
  partial_sort(vKFandScores.begin(), vKFandScores.begin() + N, vKFandScores.end());

  std::vector<StereoFrame*> vResult;
  for(unsigned int i=0; i<N; i++)
    vResult.push_back(vKFandScores[i].second);
  return vResult;
}

StereoFrame* MapMaker::ClosestKeyFrame(const StereoFrame& k)
{
  double dClosestDist = std::numeric_limits<double>::max();
  StereoFrame* closest = NULL;

  for( auto& keyFrame : map_.GetKeyFrames() ) {

    if(keyFrame == &k)
      continue;

    double dDist = KeyFrameLinearDist(k, *keyFrame);

    if(dDist < dClosestDist) {
      dClosestDist = dDist;
      closest = keyFrame;
    }
  }

  assert(closest != NULL);

  return closest;
}

void MapMaker::BundleAdjustRecent()
{
  #ifdef SHOW_PROFILING
    double start_select = GetSeg();
  #endif

  // First, make a list of the n keyframes we want adjusted in the adjuster.
  // This will be the lasts keyframe inserted
  std::vector<StereoFrame*> keyframesRefined = LastKeyFrames( nKeyFramesToAdjustByLocal_ );

  std::set<StereoFrame*> sAdjustSet;
  for ( auto keyFrame : keyframesRefined ) {
    if( not keyFrame->isFixed() ) {
        sAdjustSet.insert( keyFrame );
    }
  }

  // Now we find the set of features which they contain.
  std::set<MapPoint*> points;
  for( auto& keyFrame : sAdjustSet ) {

    for( auto& kv : keyFrame->GetMeasurementsLeft() ) {
      points.insert( kv.first );
    }

    // Get Right measurements (no problem with repetition given that we use std::set structure)
//    for( auto& kv : keyFrame->GetMeasurementsRight() ) {
//      points.insert( kv.first );
//    }
  }

  // Finally, add all keyframes which measure above points as fixed keyframes
  std::set<StereoFrame*> sFixedSet;
  for ( auto& keyFrame : map_.GetKeyFrames() ) {

    if( sAdjustSet.count( keyFrame ) ) {
      continue;
    }

    bool bInclude = false;

    for( auto& kv : keyFrame->GetMeasurementsLeft() ) {
      if( points.count( kv.first ) ) {
        bInclude = true;
        break;
      }
    }

//    if ( not bInclude ) {
//      // Get Right measurements (no problem with repetition given that we use std::set structure)
//      for( auto& kv : keyFrame->GetMeasurementsRight() ) {
//        if( points.count( kv.first ) ) {
//          bInclude = true;
//          break;
//        }
//      }
//    }

    if(bInclude)
      sFixedSet.insert( keyFrame );
  }

  #ifdef SHOW_PROFILING
    double end_select = GetSeg();
    WriteToLog(" ba local_select: ", start_select, end_select);
  #endif

  #ifdef SHOW_PROFILING
    double start_load = GetSeg();
  #endif

  std::cout << "BA Local: kf adj: " << sAdjustSet.size() << " kf fix: " <<  sFixedSet.size() << " points: " << points.size() << std::endl;

  // load the data into the bundle adjuster
  // No need to take a lock for the points, since the points are
  // aquired from the keyframe measurements and not the Map collection.
  bundleDriver_.SetData(sAdjustSet, sFixedSet, points);

  #ifdef SHOW_PROFILING
    double end_load = GetSeg();
    WriteToLog(" ba local_load: ", start_load, end_load);
  #endif

  std::cout << "EJECUTANDO BA LOCAL:" << std::endl << std::endl;

  #ifdef SHOW_PROFILING
    double start_adjust = GetSeg();
  #endif

  bool was_completed = bundleDriver_.Adjust( maxIterationsLocal_ );

  #ifdef SHOW_PROFILING
    double end_adjust = GetSeg();
    WriteToLog(" ba local_adjust: ", start_adjust, end_adjust);
  #endif

  std::cout << "DESPUES DE BA LOCAL" << std::endl << std::endl;

  #ifdef SHOW_PROFILING
    double start_save_points = GetSeg();
  #endif

  // save data even if BA was interrupted
  map_.points_mutex_.lock();

  bundleDriver_.SavePoints();

  map_.points_mutex_.unlock();

  #ifdef SHOW_PROFILING
    double end_save_points = GetSeg();
    WriteToLog(" ba local_save_points: ", start_save_points, end_save_points);
  #endif

  #ifdef SHOW_PROFILING
    double start_save_cameras = GetSeg();
  #endif

  bundleDriver_.SaveCameras();

  #ifdef SHOW_PROFILING
    double end_save_cameras = GetSeg();
    WriteToLog(" ba local_save_cameras: ", start_save_cameras, end_save_cameras);
  #endif

  #ifdef SHOW_PROFILING
    double start_handle_bad = GetSeg();
  #endif

  if ( was_completed ) {
    std::cout << "BA LOCAL CONVERGIO" << std::endl;
//    std::lock_guard<std::mutex> lock( map_.points_mutex_ );

    const std::list< std::pair<StereoFrame*, MapPoint*> >& badMeasurements = bundleDriver_.GetBadMeasurements();
    RemoveMeasurements( badMeasurements );


  } else {
    std::cout << "BA LOCAL NO CONVERGIO" << std::endl;
  }

  #ifdef SHOW_PROFILING
    double end_handle_bad = GetSeg();
    WriteToLog(" ba local_handle_bad: ", start_handle_bad, end_handle_bad);
  #endif
}

void MapMaker::BundleAdjustAll()
{
  #ifdef SHOW_PROFILING
    double start_load = GetSeg();
  #endif

  // here we only take the lock to copy the vector of MapPoints pointers
  // this is because tracker and SetData function are readers of the MapPoints
  // and that is why we do not ask the lock for the SetData function
  map_.points_mutex_.lock();
  std::vector<MapPoint*> map_copy = map_.GetMapPoints();
  map_.points_mutex_.unlock();

  // load the data into the bundle adjuster
  bundleDriver_.SetData(map_.GetKeyFrames(), map_copy);

  #ifdef SHOW_PROFILING
    double end_load = GetSeg();
    WriteToLog(" ba global_load: ", start_load, end_load);
  #endif

  // run the bundle adjuster
  std::cout << "EXECUTING GLOBAL BA:" << std::endl << std::endl;

  #ifdef SHOW_PROFILING
    double start_adjust = GetSeg();
  #endif

  bool was_completed = bundleDriver_.Adjust( maxIterationsGlobal_ );

  #ifdef SHOW_PROFILING
    double end_adjust = GetSeg();
    WriteToLog(" ba global_adjust: ", start_adjust, end_adjust);
  #endif

  std::cout << "AFTER GLOBAL BA" << std::endl << std::endl;

  // save data even if BA was interrupted

  #ifdef SHOW_PROFILING
    double start_save_points = GetSeg();
  #endif

  // save data to Map
  {
    std::lock_guard<std::mutex> lock( map_.points_mutex_ );
    bundleDriver_.SavePoints();
  }

  #ifdef SHOW_PROFILING
    double end_save_points = GetSeg();
    WriteToLog(" ba global_save_points: ", start_save_points, end_save_points);
  #endif

  #ifdef SHOW_PROFILING
    double start_save_cameras = GetSeg();
  #endif

  bundleDriver_.SaveCameras();

  #ifdef SHOW_PROFILING
    double end_save_cameras = GetSeg();
    WriteToLog(" ba global_save_cameras: ", start_save_cameras, end_save_cameras);
  #endif

  #ifdef SHOW_PROFILING
    double start_handle_bad = GetSeg();
  #endif

  if ( was_completed ) {
    // Acá entra tanto cuando llega al máximo de iteraciónes permitidas
    // como cuando se cumple el criterio de convergencia, es decir
    // la mejora en el error entre iteraciones es muy chica.
    std::cout << "BA GLOBAL CONVERGIO" << std::endl;
    const std::list< std::pair<StereoFrame*, MapPoint*> >& badMeasurements = bundleDriver_.GetBadMeasurements();
    RemoveMeasurements( badMeasurements );
  } else {
    // Acá entra cuando el BA es cortado forzadamente porque
    // tarda demasiado y hay que actualizar la información.
    // En este caso el error puede haber disminuído
    // durante las iteraciones realizadas, o no.
    std::cout << "BA GLOBAL NO CONVERGIO" << std::endl;
  }

  #ifdef SHOW_PROFILING
    double end_handle_bad = GetSeg();
    WriteToLog(" ba global_handle_bad: ", start_handle_bad, end_handle_bad);
  #endif
}

bool MapMaker::FindMeasurement(Frame& keyFrame, MapPoint& mapPoint)
{
  // abort if either a measurement is already in the map
  Measurement aux_meas;
  // TODO debe haber algo mas eficiente. Se puede chequear matchedKeypoints
  // conviene hacer todos los chequeos aca o conviene tomar precondicion?
  if( keyFrame.GetMeasurement( &mapPoint, aux_meas ) )
    return false;

  // Check if Point can be viewed by the camera
  if( not keyFrame.GetCamera().CanView( mapPoint.GetPosition() ) )
    return false;

  mapPoint.IncreaseProjectionCount();

  // Buscamos el Match
  int idx = keyFrame.FindMatch(mapPoint, *descriptorMatcher_, matchingDistanceThreshold_, matchingNeighborhood_);

  // idx < 0 signals that no match could be found
  if ( idx < 0 )
    return false;

  keyFrame.AddMeasurement( idx, mapPoint, Measurement::SRC_REFIND );

  return true;
}

std::vector<MapPoint*> MapMaker::GetNewMapPoints(const StereoFrame& keyFrame)
{

  std::vector<MapPoint*> newMapPoints;

  // TODO: thomas capaz que se puede hacer con copy_if?
  for( auto meas : keyFrame.GetMeasurementsLeft() ){
    const Measurement& measurement = meas.second;
    if ( measurement.source == Measurement::SRC_TRIANGULATION ) {
      newMapPoints.push_back( measurement.mapPoint );
    }
  }

  return newMapPoints;
}

int MapMaker::ReFindInSingleKeyFrame(StereoFrame& keyFrame)
{
  int nFoundNow = 0;

  std::lock_guard<std::mutex> lock( map_.points_mutex_ );

  for( auto mapPoint : map_.GetMapPoints() ) {

    if( FindMeasurement(keyFrame.GetFrameLeft(), *mapPoint) ) {

      nFoundNow++;
    }

    if( FindMeasurement(keyFrame.GetFrameRight(), *mapPoint) ) {

      nFoundNow++;
    }

  }

  return nFoundNow;
}

int MapMaker::ReFindNewlyMade( const std::vector<StereoFrame*>& keyFrames )
{

  if( newPoints_.empty() )
    return 0;

  int nFound = 0;
  int nBad = 0;

  // TODO: esto no sucede...
  // if a new KeyFrame is to be processed, we want to breakup
  // this loop, and continue later with the new addition
  for (auto mapPoint : newPoints_) {

    // don't look for bad points
    if( mapPoint->IsBad() ) {
      nBad++;
      continue;
    }

    for( auto keyFrame : keyFrames) {

      // refind left camera measurements
      if( FindMeasurement(keyFrame->GetFrameLeft(), *mapPoint) )
        nFound++;

      // refind right camera measurements
      if( FindMeasurement(keyFrame->GetFrameRight(), *mapPoint) )
        nFound++;
    }

  }

  // clear newPoints_ vector
  newPoints_.clear();

  return nFound;
}

void MapMaker::RemoveMeasurements( const std::list< std::pair<StereoFrame*, MapPoint*> >& badMeasurements)
{
  int count = 0;
  for (const auto& pair_kf_point : badMeasurements ) {
    StereoFrame* pk = pair_kf_point.first;
    MapPoint* pp = pair_kf_point.second;
    size_t numRemovedMeas = pk->RemoveMeasurement( pp );
    count += numRemovedMeas;
    forn ( i, numRemovedMeas ) {
      pp->IncreaseOutlierCount();
    }
  }

  std::cout << "mediciones borradas por el BA: " << count << std::endl;
}

// TODO: ESTA FUNCION NO DEBERIA IR ACA
bool InitFromStereo(
  sptam::Map& map,
  StereoFrame& frame,
  const cv::Mat& imageLeft,
  const RowMatcher& matcher)
{
  // Triangulate evenly distributed features

  std::vector<cv::Point3d> points;
  std::vector<cv::Point2d> featuresLeft, featuresRight;
  std::vector<cv::Mat> descriptorsLeft, descriptorsRight;

  frame.TriangulatePoints(
    matcher,
    points,
    featuresLeft, descriptorsLeft,
    featuresRight, descriptorsRight
  );

  // check that there are at least a minimum number of correct matches when there is no map
  if (points.empty() || (points.size() < 10 && map.nMapPoints() == 0)) {
    return false;
  }

  forn ( i, points.size() ) {

    cv::Point3d normal = points[ i ] - frame.GetPosition();
    normal = normal * ( 1 / cv::norm(normal) );

    // Create Map Point and save in map
    MapPoint* mapPoint = new MapPoint( points[ i ], normal, descriptorsLeft[ i ] );
    mapPoint->color = imageLeft.at<cv::Vec3b>( featuresLeft[ i ] );

    // hay que pedir el lock cada vez que se modifica el mapa
    map.points_mutex_.lock();
    map.AddMapPoint( mapPoint );
    map.points_mutex_.unlock();

    Measurement measLeft(mapPoint, featuresLeft[i], descriptorsLeft[i]);
    measLeft.source = Measurement::SRC_TRIANGULATION;

    Measurement measRight(mapPoint, featuresRight[i], descriptorsRight[i]);
    measRight.source = Measurement::SRC_TRIANGULATION;

    frame.AddMeasurementLeft( measLeft );
    frame.AddMeasurementRight( measRight );
  }

  std::cout << "Points initialized from stereo: " << map.nMapPoints() << std::endl;

  // Add Keyframe to the map
  // TODO esto es peligroso si no nos pasan algo que viva en el heap.
  // Es preferible tomar un shared pointer (en esta funcion ya que estamos)
  map.AddKeyFrame( &frame );

  return true;
}
