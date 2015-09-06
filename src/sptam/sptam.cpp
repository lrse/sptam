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

#include "sptam.hpp"
#include "utils/macros.hpp"
#include "KeyFramePolicy.hpp"
#include "FeatureExtractorThread.hpp"

#ifdef SHOW_TRACKED_FRAMES

  #include "utils/Draw.hpp"

  // window to draw tracking image
  #define KEYFRAME_TRACKER_WINDOW_NAME "Tracker KeyFrame"

#endif // SHOW_TRACKED_FRAMES

#ifdef SHOW_PROFILING
  #include "../sptam/utils/Profiler.hpp"
#endif // SHOW_PROFILING

SPTAM::SPTAM(
  sptam::Map& map,
  const CameraParameters& cameraParametersLeft,
  const CameraParameters& cameraParametersRight,
  const double stereo_baseline,
  const RowMatcher& rowMatcher,
  const MapMaker::Parameters& params
)
  : map_( map )
  , mapMaker_( map_, cameraParametersLeft, cameraParametersRight, stereo_baseline, params )
  , tracker_( cameraParametersLeft.intrinsic, cameraParametersRight.intrinsic, stereo_baseline )
  , params_( params )
  , cameraParametersLeft_( cameraParametersLeft )
  , cameraParametersRight_( cameraParametersRight )
  , stereo_baseline_( stereo_baseline )
  , rowMatcher_( rowMatcher )
  , frames_since_last_kf_( 0 )
  , frames_number_ ( 0 )
{}

CameraPose SPTAM::track(
  const CameraPose& estimatedCameraPose,
  const ImageFeatures& imageFeaturesLeft, const ImageFeatures& imageFeaturesRight,
  const cv::Mat &imageLeft, const cv::Mat &imageRight)
{
  StereoFrame::UniquePtr frame( new StereoFrame(
    estimatedCameraPose, cameraParametersLeft_,
    stereo_baseline_, cameraParametersRight_,
    imageFeaturesLeft, imageFeaturesRight
  ));

  // Here we will save the measured features in this frame,
  // to be passed to the tracker for pose refinement.
  std::vector<Measurement> measurementsLeft, measurementsRight;

  // Try to match the features found in the new frame to the
  // 3D points saved in the map.
  matchToGlobal( *frame, measurementsLeft, measurementsRight );

  #ifdef SHOW_PROFILING
    WriteToLog( " tk matched_feat_left: ", measurementsLeft.size() );
    WriteToLog( " tk matched_feat_right: ", measurementsRight.size() );
    WriteToLog( " tk matched_feat_total: ", measurementsLeft.size() + measurementsRight.size() );
  #endif

  #ifdef SHOW_PROFILING
    double startStep, endStep;
    startStep = GetSeg();
  #endif

  // Load map point measurements into the new frame

  for ( auto meas : measurementsLeft ) {
    frame->AddMeasurementLeft( meas );
  }

  for ( auto meas : measurementsRight ) {
    frame->AddMeasurementRight( meas );
  }

  #ifdef SHOW_TRACKED_FRAMES
    // show tracking results as measurement / reprojection correspondences
    // for the stereo frame (left is the only one used for tracking)
    cv::Mat trackingFrame = drawTrackedFrames( *frame, imageLeft, imageRight);
    cv::imshow(KEYFRAME_TRACKER_WINDOW_NAME, trackingFrame);
    cv::waitKey(30);
  #endif // SHOW_TRACKED_FRAMES

  std::map<MapPoint*, Measurement> measurementsLeftOnly, emptyMap;
  std::map<MapPoint*, std::pair<Measurement, Measurement> > measurementsStereo;
  measurementsStereo = frame->GetMeasurementsStereo();
  measurementsLeftOnly = frame->GetMeasurementsLeftOnly();
//  measurementsRightOnly = frame->GetMeasurementsRightOnly(); // las mediciones derechas no estan andando bien

  // The tracker will try to refine the new camera pose
  // from the computed measurements
  CameraPose refinedCameraPose = tracker_.RefineCameraPose(
    estimatedCameraPose, measurementsStereo, measurementsLeftOnly, emptyMap
  );

  #ifdef SHOW_PROFILING
    std::map<MapPoint*, Measurement> measurementsRightOnly = frame->GetMeasurementsRightOnly();
    WriteToLog( " tk matched_points_stereo: ", measurementsStereo.size() );
    WriteToLog( " tk matched_points_only_left: ", measurementsLeftOnly.size() );
    WriteToLog( " tk matched_points_only_right: ", measurementsRightOnly.size() );
  #endif

  // Update frame pose
  frame->UpdateCameraPose( refinedCameraPose );

  #ifdef SHOW_PROFILING
    endStep = GetSeg();
    WriteToLog(" tk tracking_refine: ", startStep, endStep);
  #endif

  frames_since_last_kf_++;
  frames_number_++;

  #ifdef SHOW_TRACKED_FRAMES
    cv::Mat updatedFrame = drawTrackedFrames( *frame, imageLeft, imageRight);
    cv::imshow("After Refine", updatedFrame);
    cv::waitKey(30);
  #endif // SHOW_TRACKED_FRAMES

  if( shouldBeKeyframe( *frame ) ) {

    frame->SetId( frames_number_ );

    // Create new 3D points from unmatched features,
    // and save them in the local tracking map.
    std::vector<MapPoint*> newPoints;

    #ifdef SHOW_PROFILING
      startStep = GetSeg();
    #endif

    createNewPoints( *frame, imageLeft, newPoints );

    #ifdef SHOW_PROFILING
      WriteToLog(" tk created_new_points: ", newPoints.size());
    #endif

    #ifdef SHOW_PROFILING
      endStep = GetSeg();
      WriteToLog(" tk CreatePoints: ", startStep, endStep);
    #endif

    // add new points to the global map

    {
      #ifdef SHOW_PROFILING
        startStep = GetSeg();
      #endif

      std::lock_guard<std::mutex> lock( map_.points_mutex_ );

      #ifdef SHOW_PROFILING
        endStep = GetSeg();
        WriteToLog(" tk LockAddPoints: ", startStep, endStep);
      #endif

      map_.AddMapPoints( newPoints );
    }

    #ifdef SHOW_PROFILING
      startStep = GetSeg();
    #endif

    // The mapMaker takes ownership of the UniquePtr,
    // so after this call the variable should not be used.
    mapMaker_.AddKeyFrame( frame );

    #ifdef SHOW_PROFILING
      endStep = GetSeg();
      WriteToLog(" tk AddKeyFrame: ", startStep, endStep);
    #endif

    frames_since_last_kf_ = 0;

    std::cout << " Adding key-frame." << std::endl;
  }
  #ifdef SHOW_PROFILING
  else {
    std::cout << "frames since last kf: " << frames_since_last_kf_ << std::endl;
    // Esto está para que haya tantos mensajes como frames
    WriteToLog(" tk CreatePoints: ", 0, 0);
    WriteToLog(" tk LockAddPoints: ", 0, 0);
    WriteToLog(" tk AddKeyFrame: ", 0, 0);
  }
  #endif

  return refinedCameraPose;
}

void SPTAM::matchToGlobal( const StereoFrame& frame,
                          std::vector<Measurement>& measurementsLeft,
                          std::vector<Measurement>& measurementsRight)
{
  std::vector<MapPoint*> filtered_points;

  #ifdef SHOW_PROFILING
    double start, end;
    start = GetSeg();
  #endif

  // std::cout << "lockeando mapa para frustum culling" << std::endl;
  map_.points_mutex_.lock();

  #ifdef SHOW_PROFILING
    end = GetSeg();
    WriteToLog(" tk LockFrustum: ", start, end);
  #endif

  #ifdef SHOW_PROFILING
  start = GetSeg();
  #endif

  // Filter map points by frustum culling
  // TODO: esto se le deberia poder pedir al mapa, y el mapa
  // lo debería devolver de manera eficiente (si le fuese posible).
  for ( auto mapPoint : map_.GetMapPoints() ) {

    const cv::Point3d& point = mapPoint->GetPosition();

    // Compute current normal
    const cv::Point3d& cameraPosition = frame.GetPosition();

    cv::Point3d currentNormal = point - cameraPosition;
    currentNormal = currentNormal * ( 1 / cv::norm( currentNormal ) );

    const cv::Point3d& pointNormal = mapPoint->GetNormal();

    // angle is in radians
    double angle = std::acos( pointNormal.dot( currentNormal ) );

    // Discard Points which were created from a greater 45 degrees pint of view.
    // TODO: pass as parameter
    if ( angle < (M_PI / 4.0) ) {

      bool isPointViewed = false;

      if ( frame.GetCameraLeft().CanView( point ) ) {
        mapPoint->IncreaseProjectionCount(); // increase by projection counter
        isPointViewed = true;
      }
      if ( frame.GetCameraRight().CanView( point ) ) {
        mapPoint->IncreaseProjectionCount(); // increase by projection counter
        isPointViewed = true;
      }
      if ( isPointViewed ) {
        filtered_points.push_back( mapPoint );
      }
    }
  }

  #ifdef SHOW_PROFILING
    end = GetSeg();
    WriteToLog(" tk Frustum: ", start, end);
    WriteToLog(" tk visiblePoints: ", filtered_points.size());
  #endif

  map_.points_mutex_.unlock();

  matchToPoints(frame, filtered_points, measurementsLeft, measurementsRight);
}

void SPTAM::matchToPoints(const StereoFrame& frame, const std::vector<MapPoint*>& mapPoints,
                         std::vector<Measurement>& measurementsLeft, std::vector<Measurement>& measurementsRight)
{
  std::vector<cv::Point3d> points;
  std::vector<cv::Mat> descriptors;

  points.reserve( mapPoints.size() );
  descriptors.reserve( mapPoints.size() );

  for ( auto mapPoint : mapPoints ) {
    points.push_back( mapPoint->GetPosition() );
    descriptors.push_back( mapPoint->GetDescriptor() );
  }

  #ifdef SHOW_PROFILING
    double start, end;
    start = GetSeg();
  #endif

  std::vector<MEAS> measLeft, measRight;

  frame.FindMatches(
    points, descriptors,
    *params_.descriptorMatcher,
    params_.matchingNeighborhoodThreshold,
    params_.matchingDistanceThreshold,
    measLeft, measRight
  );

  #ifdef SHOW_PROFILING
    end = GetSeg();
    WriteToLog(" tk FindMatches: ", start, end);
  #endif

  measurementsLeft.reserve( measLeft.size() );
  measurementsRight.reserve( measRight.size() );

  for ( auto meas : measLeft ) {

    // Update the descriptor of the MapPoint with the left image descriptor only
    // aca haria falta pedir el lock
    mapPoints[ meas.index ]->SetDescriptor( meas.descriptor );

    Measurement measurement(mapPoints[ meas.index ], meas.projection, meas.descriptor);
    measurement.source = Measurement::SRC_TRACKER;

    measurementsLeft.push_back( measurement );
  }

  for ( auto meas : measRight ) {

    Measurement measurement(mapPoints[ meas.index ], meas.projection, meas.descriptor);
    measurement.source = Measurement::SRC_TRACKER;

    measurementsRight.push_back( measurement );
  }
}

void SPTAM::createNewPoints(
  StereoFrame& frame,
  const cv::Mat& imageLeft,
  std::vector<MapPoint*>& mapPoints)
{
  std::vector<cv::Point3d> points;
  std::vector<cv::Point2d> featuresLeft, featuresRight;
  std::vector<cv::Mat> descriptorsLeft, descriptorsRight;

  frame.TriangulatePoints(
    rowMatcher_,
    points,
    featuresLeft, descriptorsLeft,
    featuresRight, descriptorsRight
  );

  forn( i, points.size() ) {

    cv::Point3d normal = points[ i ] - frame.GetPosition();
    normal = normal * ( 1 / cv::norm(normal) );

    MapPoint* mapPoint = new MapPoint( points[ i ], normal, descriptorsLeft[ i ] );
    mapPoint->color = imageLeft.at<cv::Vec3b>( featuresLeft[ i ] );

    mapPoints.push_back( mapPoint );

    Measurement measLeft(mapPoint, featuresLeft[i], descriptorsLeft[i]);
    measLeft.source = Measurement::SRC_TRIANGULATION;

    Measurement measRight(mapPoint, featuresRight[i], descriptorsRight[i]);
    measRight.source = Measurement::SRC_TRIANGULATION;


    frame.AddMeasurementLeft( measLeft );
    frame.AddMeasurementRight( measRight );
  }
}

bool SPTAM::shouldBeKeyframe(const StereoFrame& frame)
{

  #ifdef SHOW_PROFILING
    double start, end;
    start = GetSeg();
  #endif

  // las Heuristicas de agregado de keyframes es un trade-off entre velocidad y precision
  // Mientras mas esporadicamente se agregan los keyframes,
  // se tendra mas tiempo para ajustar el mapa, pero menos preciso sera el tracking

  // Based Distance Policy
//  return AddKeyFrameDistancePolicy(frame, map_, params_.keyFrameDistanceThreshold)
//      or (params_.framesBetweenKeyFrames < frames_since_last_kf_);

  // Based free coverage image Policy
//  return AddKeyFrameImageCoverPolicy(frame, 1241, 0.2);

  // Based Matched Features percentage Policy
//  return AddKeyFrameFeaturesPolicy(frame, 0.1);


  // Based Tracking features percentage wrt to reference keyframe Policy
  size_t numStereoMeas = frame.GetMeasurementsStereo().size();
  bool shouldBeKeyframe = (AddKeyFrameTrackingFeaturesPolicy(frame, map_, 0.9)
                          or (numStereoMeas < 20)); // number of matches less than a threshold

  #ifdef SHOW_PROFILING
    end = GetSeg();
    WriteToLog(" tk keyframe_selection_strategy: ", start, end);
  #endif

  return shouldBeKeyframe;

}
