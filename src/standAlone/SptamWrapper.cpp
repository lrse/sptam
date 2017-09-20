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

#include "SptamWrapper.hpp"
#include <boost/chrono.hpp>
#include <boost/thread/thread.hpp>

#include "../sptam/utils/macros.hpp"
#include "../sptam/TrackerViewStereo.hpp"

#ifdef SHOW_PROFILING
#include "../sptam/utils/log/Profiler.hpp"
#include "../sptam/utils/log/ScopedProfiler.hpp"
#endif

SptamWrapper::SptamWrapper(const CameraParameters& cameraParametersLeft,
  const CameraParameters& cameraParametersRight,
  const double stereo_baseline, const RowMatcher& rowMatcher,
  const Parameters& params,
  std::shared_ptr<PosePredictor> motionModel,
  const size_t imageBeginIndex
)
  : cameraParametersLeft_( cameraParametersLeft )
  , cameraParametersRight_( cameraParametersRight )
  , stereo_baseline_( stereo_baseline )
  , rowMatcher_( rowMatcher )
  , params_( params )
  , motionModel_( motionModel )
  , isMapInitialized_( false )
  , sptam_(rowMatcher, params)
{}

#ifdef USE_LOOPCLOSURE
void SptamWrapper::setLoopClosing(std::unique_ptr<LCDetector>& loop_detector)
{
  sptam_.setLoopClosing(loop_detector);
}
#endif

void SptamWrapper::Add(const size_t frame_id, const ros::Time& time, std::unique_ptr<StereoImageFeatures> stereoImageFeatures)
{
  ImageFeatures& imageFeaturesLeft = stereoImageFeatures->imageFeaturesLeft;
  ImageFeatures& imageFeaturesRight = stereoImageFeatures->imageFeaturesRight;

  #ifdef SHOW_PROFILING
    sptam::ScopedProfiler timer(" tk trackingWithoutExtraction: ");
  #endif

  Eigen::Vector3d estimatedCameraPosition;
  Eigen::Quaterniond estimatedCameraOrientation;
  Eigen::Matrix6d predictionCovariance;

  motionModel_->predictPose( time, estimatedCameraPosition, estimatedCameraOrientation, predictionCovariance );
  const CameraPose estimatedCameraPose( estimatedCameraPosition, estimatedCameraOrientation, predictionCovariance );

  StereoFrame frame(
    frame_id,
    estimatedCameraPose, cameraParametersLeft_,
    stereo_baseline_, cameraParametersRight_,
    imageFeaturesLeft, imageFeaturesRight, not isMapInitialized_
  );

  // aca me guardo el resultado si la CameraPose es refinada por el tracker.
  CameraPose refined_camera_pose = estimatedCameraPose;

  // Check is there is no initial map create it
  if (not isMapInitialized_)
  {
    // Initialize MapMaker
    try {
      sptam_.init( frame );
      isMapInitialized_ = true;
    }
    catch( std::runtime_error& err ) {
      std::cerr << err.what() << std::endl;
    }
  }
  // if there is an initial map then run SPTAM
  else
  {
    #ifdef SHOW_PROFILING
    {
      const CameraPose& camera_pose = frame.GetCameraPose();
      writePoseToLog("ESTIMATED_CAMERA_POSE", frame_id, camera_pose.GetPosition(), camera_pose.GetOrientationMatrix(), camera_pose.covariance());
    }
    #endif

    sptam::TrackerViewStereo tracker_view(stereoImageFeatures->imageLeft, stereoImageFeatures->imageRight);
    tracker_view.enableDrawOutput((sptam::TrackerViewStereo::DrawOutput)(sptam::TrackerViewStereo::DRAW_AFTER_REFINE_STEREO | sptam::TrackerViewStereo::DRAW_BEFORE_REFINE_STEREO));

    TrackingReport report = sptam_.track(frame, tracker_view);

    refined_camera_pose = report.refinedCameraPose;

    // MotionModel needs to be notify of this abrupt correction
    if (!report.T_corr.isIdentity() && motionModel_ != nullptr) // nothing to correct
        motionModel_->applyCorrection(report.T_corr);

    #ifdef SHOW_PROFILING
      writePoseToLog("REFINED_CAMERA_POSE", frame_id, refined_camera_pose.GetPosition(), refined_camera_pose.GetOrientationMatrix(), refined_camera_pose.covariance() );
    #endif

    #ifdef SHOW_TRACKED_FRAMES
    cv::imshow("Before Refine", tracker_view.stereoFrameBeforeRefine);
    cv::imshow("After Refine", tracker_view.stereoFrameAfterRefine);
    cv::waitKey( 1 );
    #endif
  } // else

  #ifdef SHOW_PROFILING
    writePoseToLog("TRACKED_FRAME_POSE", frame_id, refined_camera_pose.GetPosition(), refined_camera_pose.GetOrientationMatrix(), refined_camera_pose.covariance());
  #endif

  // Update motion model
  motionModel_->updatePose(time, refined_camera_pose.GetPosition(), refined_camera_pose.GetOrientationQuaternion(), refined_camera_pose.covariance());
}
