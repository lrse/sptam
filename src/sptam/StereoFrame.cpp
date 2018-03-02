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

#include "StereoFrame.hpp"
#include "RowMatcher.hpp"
#include "utils/cv2eigen.hpp"
#include "utils/projective_math.hpp"
#include "utils/timer.h"

#if SHOW_PROFILING
#include "utils/log/Profiler.hpp"
#endif

#include <opencv2/calib3d/calib3d.hpp>  // cv::triangulatePoints
#include <opencv2/highgui/highgui.hpp>  // for debugging
#include <thread>

StereoFrame::StereoFrame(
  const size_t id,
  const CameraPose& cameraPose, const CameraParameters& calibrationLeft,
  const double stereo_baseline, const CameraParameters& calibrationRight,
  const ImageFeatures& imageFeaturesLeft, const ImageFeatures& imageFeaturesRight,
  bool bFixed
)
  : frameLeft_(Camera(cameraPose, calibrationLeft), imageFeaturesLeft)
  , frameRight_(Camera(ComputeRightCameraPose(cameraPose, stereo_baseline), calibrationRight), imageFeaturesRight)
  , rectified_camera_parameters_({stereo_baseline, calibrationLeft.focalLengths(), calibrationLeft.principalPoint()})
  , bFixed_( bFixed ), id_( id )
{}

/* When a keyframe is added to the Map, has to be copyed, but the shared_mutex does not have a
 * a default copy constructor. Thats why we have to specify that the new points will have a separate mutex */
StereoFrame::StereoFrame(const StereoFrame& stereoFrame)
  : stereo_frames_mutex_() // The copy has a diferent mutex!
  , frameLeft_( stereoFrame.frameLeft_ ) // TODO: check if Frame copy constructor is working properly.
  , frameRight_( stereoFrame.frameRight_ ) // TODO: check if Frame copy constructor is working properly.
  , rectified_camera_parameters_( stereoFrame.rectified_camera_parameters_ )
  , bFixed_( stereoFrame.bFixed_ )
  , id_( stereoFrame.id_ )
{}

#define PROFILE_INTERNAL_MATCH 0

void StereoFrame::FindMatches(const Measurement::Source source,
  const std::aligned_vector<Eigen::Vector3d>& points, const std::vector<cv::Mat>& descriptors,
  const cv::DescriptorMatcher& descriptorMatcher,
  const size_t matchingNeighborhoodThreshold, const double matchingDistanceThreshold,
  std::list<size_t>& matchedIndexes, std::list<Measurement>& measurements
) const
{
  boost::unique_lock<boost::shared_mutex> lock(stereo_frames_mutex_);

  // TODO: maybe is useful to pass right descriptors too

  std::list<std::pair<size_t, size_t> > measurementsLeft, measurementsRight;

#if 1
  /* parallel find match */
  #if SHOW_PROFILING && PROFILE_INTERNAL_MATCH
  sptam::Timer t_left, t_right;
  #endif

  #if SHOW_PROFILING && PROFILE_INTERNAL_MATCH
  t_left.start();
  #endif

  std::thread thread_left([&]() { measurementsLeft = frameLeft_.FindMatches(points, descriptors, descriptorMatcher, matchingDistanceThreshold, matchingNeighborhoodThreshold);
  #if SHOW_PROFILING && PROFILE_INTERNAL_MATCH
    t_left.stop();
  #endif
  });

  #if SHOW_PROFILING && PROFILE_INTERNAL_MATCH
  t_right.start();
  #endif

  std::thread thread_right([&]() { measurementsRight = frameRight_.FindMatches(points, descriptors, descriptorMatcher, matchingDistanceThreshold, matchingNeighborhoodThreshold);
  #if  SHOW_PROFILING && PROFILE_INTERNAL_MATCH
    t_right.stop();
  #endif
  });
  thread_left.join();
  thread_right.join();

  #if SHOW_PROFILING && PROFILE_INTERNAL_MATCH
  WriteToLog(" xx findMatches-left: ", t_left);
  WriteToLog(" xx findMatches-right: ", t_right);
  #endif
#else
  /* sequential find match */
  measurementsLeft = frameLeft_.FindMatches(points, descriptors, descriptorMatcher, matchingDistanceThreshold, matchingNeighborhoodThreshold);
  measurementsRight = frameRight_.FindMatches(points, descriptors, descriptorMatcher, matchingDistanceThreshold, matchingNeighborhoodThreshold);
#endif

  #if SHOW_PROFILING
  sptam::Timer t_process;
  t_process.start();
  #endif

  // This is messy but efficient

  auto it_left = measurementsLeft.begin();
  auto it_right = measurementsRight.begin();

  // iterate while there is something to be added to matches.
  while ( it_left != measurementsLeft.end() or it_right != measurementsRight.end() )
  {
    // if both have the same source mapPoint, add a stereo measurement.
    if ( it_left != measurementsLeft.end() and it_right != measurementsRight.end() and it_left->first == it_right->first )
    {
      // Create the Measurement
      measurements.push_back( Measurement(source,
        frameLeft_.GetFeatures().GetKeypoint( it_left->second ), frameLeft_.GetFeatures().GetDescriptor( it_left->second ),
        frameRight_.GetFeatures().GetKeypoint( it_right->second ), frameRight_.GetFeatures().GetDescriptor( it_right->second )
      ));

      frameLeft_.SetMatchedKeyPoint( it_left->second );
      frameRight_.SetMatchedKeyPoint( it_right->second );

      matchedIndexes.push_back( it_left->first );

      ++ it_left;
      ++ it_right;
    }
    // if the next index to process is the left one.
    else if ( it_right == measurementsRight.end() or ( it_left != measurementsLeft.end() and it_left->first < it_right->first ) )
    {
      // Create the Measurement
      measurements.push_back( Measurement(Measurement::LEFT, source, frameLeft_.GetFeatures().GetKeypoint( it_left->second ), frameLeft_.GetFeatures().GetDescriptor( it_left->second )) );

      frameLeft_.SetMatchedKeyPoint( it_left->second );

      matchedIndexes.push_back( it_left->first );

      ++ it_left;
    }
    // if the next index to process is the right one.
    else
    {
      measurements.push_back( Measurement(Measurement::RIGHT, source, frameRight_.GetFeatures().GetKeypoint( it_right->second ), frameRight_.GetFeatures().GetDescriptor( it_right->second )) );

      frameRight_.SetMatchedKeyPoint( it_right->second );

      matchedIndexes.push_back( it_right->first );

      ++ it_right;
    }
  }

  #if SHOW_PROFILING
  t_process.stop();
  WriteToLog(" XX findMatches-process: ", t_process);
  #endif
}

#define PROFILE_INTERNAL_TRIANGULATE 0

void StereoFrame::TriangulatePoints(const RowMatcher& matcher, std::aligned_vector<MapPoint> &points, std::vector<Measurement> &measurements)/* const*/
{
#if defined(SHOW_PROFILING) && PROFILE_INTERNAL_TRIANGULATE
  sptam::Timer t_lock, t_create;
  t_lock.start();
#endif

  boost::unique_lock<boost::shared_mutex> lock(stereo_frames_mutex_);

#if defined(SHOW_PROFILING) && PROFILE_INTERNAL_TRIANGULATE
  t_lock.stop();
#endif

  // TODO ImageFeatures podria guardar internamente el vector
  // de los unmatched, y descartar los matched, total nunca más los uso.
  // Esto permitiría recibir acá una referencia

  // retrieve unmatched features from both frames.

  std::vector<cv::KeyPoint> unmatchedKeypointsLeft, unmatchedKeypointsRight;
  cv::Mat unmatchedDescriptorsLeft, unmatchedDescriptorsRight;
  std::vector<size_t> indexesLeft, indexesRight;

  // Get descritptors for both images

  frameLeft_.GetUnmatchedKeyPoints(unmatchedKeypointsLeft, unmatchedDescriptorsLeft, indexesLeft);

  frameRight_.GetUnmatchedKeyPoints(unmatchedKeypointsRight, unmatchedDescriptorsRight, indexesRight);

  std::list<std::pair<size_t, size_t>> matches;

#if defined(SHOW_PROFILING) && PROFILE_INTERNAL_TRIANGULATE
  t_create.start();
#endif

  CreatePoints(
    matcher,
    unmatchedKeypointsLeft, unmatchedDescriptorsLeft,
    unmatchedKeypointsRight, unmatchedDescriptorsRight,
    points, matches
  );

#if defined(SHOW_PROFILING) && PROFILE_INTERNAL_TRIANGULATE
  t_create.stop();
#endif

  measurements.reserve( matches.size() );

  for ( auto pair : matches )
  {
    size_t idxLeft = pair.first;
    size_t idxRight = pair.second;

    frameLeft_.SetMatchedKeyPoint( indexesLeft[ idxLeft ] );
    frameRight_.SetMatchedKeyPoint( indexesRight[ idxRight] );

    measurements.push_back( Measurement(
      Measurement::SRC_TRIANGULATION,
      unmatchedKeypointsLeft[ idxLeft ], unmatchedDescriptorsLeft.row( idxLeft ),
      unmatchedKeypointsRight[ idxRight ], unmatchedDescriptorsRight.row( idxRight )
    ) );
  }

#if defined(SHOW_PROFILING) && PROFILE_INTERNAL_TRIANGULATE
  WriteToLog(" xx triangulate_points-lockStereo: ", t_lock);
  WriteToLog(" xx triangulate_points-create: ", t_create);
#endif
}

#define PROFILE_INTERNAL_CREATE 0

void StereoFrame::CreatePoints(
  const RowMatcher& matcher,
  const std::vector<cv::KeyPoint>& keypointsLeft, const cv::Mat& allDescriptorsLeft,
  const std::vector<cv::KeyPoint>& keypointsRight, const cv::Mat& allDescriptorsRight,
  std::aligned_vector<MapPoint>& points, std::list<std::pair<size_t, size_t>>& matches
)
{
#if defined(SHOW_PROFILING) && PROFILE_INTERNAL_CREATE
  sptam::Timer t_match, t_triangulate;
  t_match.start();
#endif

  // per-row matcher for stereo rectify images
  std::vector<cv::DMatch> cvMatches;
  matcher.match(keypointsLeft, allDescriptorsLeft, keypointsRight, allDescriptorsRight, cvMatches);

#if defined(SHOW_PROFILING) && PROFILE_INTERNAL_CREATE
  t_match.stop();
#endif

  // Return if no matches were found
  if ( cvMatches.empty() ) {
    std::cerr << "No matches found for triangulation" << std::endl;
    return;
  }

  const Camera& cameraLeft = frameLeft_.GetCamera();
  const Camera& cameraRight = frameRight_.GetCamera();

  // Compute Projections matrices
  cv::Matx34d projectionLeft = eigen2cv( cameraLeft.GetProjection() );
  cv::Matx34d projectionRight = eigen2cv( cameraRight.GetProjection() );

  const std::vector<cv::DMatch>& goodMatches = cvMatches;
  //std::cout << "F: Correct (by F) / Total matches: " << goodMatches.size() << " / " << cvMatches.size() << std::endl;

  std::vector<cv::Point2d> matchedPointsLeft;
  std::vector<cv::Point2d> matchedPointsRight;
  matchedPointsLeft.reserve( goodMatches.size() );
  matchedPointsRight.reserve( goodMatches.size() );

  // initialize the points
  for ( auto match : goodMatches ) {
    matchedPointsLeft.push_back( keypointsLeft[ match.queryIdx ].pt );
    matchedPointsRight.push_back( keypointsRight[ match.trainIdx ].pt );
  }

#if defined(SHOW_PROFILING) && PROFILE_INTERNAL_CREATE
  t_triangulate.start();
#endif

  // Triangulate 3D points from both cameras
  cv::Mat point3DHomos;
  cv::triangulatePoints(
    projectionLeft, projectionRight, matchedPointsLeft, matchedPointsRight, point3DHomos
  );

#if defined(SHOW_PROFILING) && PROFILE_INTERNAL_CREATE
  t_triangulate.stop();
#endif

  // Filter some more by viewing frustum and fill the return parameters
  for(int i = 0; i < point3DHomos.cols; ++i)
  {
    Eigen::Vector3d point = cv2eigen( toInHomo(cv::Vec4d( point3DHomos.col(i) )) );

    // if the point is not in range of any camera it is discarded
    if( not cameraLeft.CanView( point ) || not cameraRight.CanView( point ) )
      continue;

    // this indexes are for the original unmatched collections
    size_t idxLeft = goodMatches[i].queryIdx;
    size_t idxRight = goodMatches[i].trainIdx;

    Eigen::Vector3d normal = point - frameLeft_.GetPosition();
    normal.normalize();

    points.push_back( MapPoint( point, normal, allDescriptorsLeft.row( idxLeft ), INITIAL_POINT_COVARIANCE ) );

    matches.push_back( std::pair<size_t, size_t>(idxLeft, idxRight) );
  }

#if defined(SHOW_PROFILING) && PROFILE_INTERNAL_CREATE
  WriteToLog(" xx triangulate_points-create-match ", t_match);
  WriteToLog(" xx triangulate_points-create-triangulate ", t_triangulate);
#endif
}

void StereoFrame::UpdateCameraPose(const CameraPose& cameraPose)
{
  boost::unique_lock<boost::shared_mutex> lock(stereo_frames_mutex_);

  frameLeft_.UpdateCameraPose( cameraPose );
  frameRight_.UpdateCameraPose( ComputeRightCameraPose(cameraPose, rectified_camera_parameters_.baseline) );
}

bool StereoFrame::canView(const MapPoint& mapPoint) const
{
  boost::shared_lock<boost::shared_mutex> lock(stereo_frames_mutex_);

  const Eigen::Vector3d& point = mapPoint.GetPosition();

  // compute only once, since both cameras have the same orientation.
  bool similar_angle = false;
  {
    Eigen::Vector3d currentNormal = point - frameLeft_.GetPosition();
    currentNormal.normalize();

    // angle is in radians
    double angle = std::acos( ( mapPoint.GetNormal() ).dot( currentNormal ) );

    // Discard Points which were created from a greater 45 degrees pint of view.
    // TODO: pass this threshold as a parameter.
    similar_angle = angle < (M_PI / 4.0);
  }

  return similar_angle and ( frameLeft_.GetCamera().CanView( point ) or frameRight_.GetCamera().CanView( point ) );
}

  /*** private functions ***/

CameraPose StereoFrame::ComputeRightCameraPose(const CameraPose& leftCameraPose, const double stereo_baseline)
{
  // The position is the baseline converted to world coordinates.
  // The orientation is the same as the left camera.
  return CameraPose(leftCameraPose.ToWorld( Eigen::Vector3d(stereo_baseline, 0, 0) ), leftCameraPose.GetOrientationQuaternion(), leftCameraPose.covariance());
}
