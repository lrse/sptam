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

#include "stereo_driver.hpp"
#include "utils/opencv_parsers.hpp"
#include "../sptam/FeatureExtractorThread.hpp"
#include "../sptam/TrackerViewStereo.hpp"
#include "../sptam/utils/cv2eigen.hpp"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <image_geometry/stereo_camera_model.h>
#include <image_geometry/pinhole_camera_model.h>

#ifdef USE_LOOPCLOSURE
#include "../sptam/loopclosing/LoopClosing.hpp"
#include "../sptam/loopclosing/LCDetector.hpp"
#include "../sptam/loopclosing/detectors/DLDLoopDetector.hpp"
#endif

#ifdef SHOW_PROFILING
  #include "../sptam/utils/log/Profiler.hpp"
  #include "../sptam/utils/log/Logger.hpp"
#endif // SHOW_PROFILING

sptam::stereo_driver::stereo_driver(ros::NodeHandle& nh, ros::NodeHandle& nhp)
  : base_driver(nh, nhp)
  , cameraParametersLeft_( nullptr ), cameraParametersRight_( nullptr )
  , lastImageSeq_( 0 ), imgTransport_( nhp )
{
  // Get node parameters

  bool use_approx_sync;
  nhp.param<bool>("approximate_sync", use_approx_sync, false);

  // Camera Calibration Parameters

  nhp.param<double>("FrustumNearPlaneDist", frustum_near_plane_distance_, 0.1);
  nhp.param<double>("FrustumFarPlaneDist", frustum_far_plane_distance_, 1000.0);

  // Load feature detectors and descriptors

  // Load salient point detector implementation from configuration.
  std::string detector_name;
  nhp.param<std::string>("FeatureDetector/Name", detector_name, "GFTT");

  feature_detector_left_ = loadFeatureDetector( nhp, detector_name, "FeatureDetector" );
  feature_detector_right_ = loadFeatureDetector( nhp, detector_name, "FeatureDetector" );

  // Load descriptor extractor implementation from configuration.
  std::string descriptor_name;
  nhp.param<std::string>("DescriptorExtractor/Name", descriptor_name, "BRIEF");

  descriptor_extractor_left_ = loadDescriptorExtractor( nhp, descriptor_name, "DescriptorExtractor" );
  descriptor_extractor_right_ = loadDescriptorExtractor( nhp, descriptor_name, "DescriptorExtractor" );

  // write configuration to log file
  #ifdef SHOW_PROFILING
    Logger::Write( "#   FrustumNearPlaneDist: " + std::to_string(  frustum_near_plane_distance_ ) + "\n" );
    Logger::Write( "#   FrustumFarPlaneDist: " + std::to_string(  frustum_far_plane_distance_ ) + "\n" );
  #endif

  #ifdef USE_LOOPCLOSURE
  loop_detector_ = nullptr;
  // Loop detector vocabulary parameters
  if (nhp.hasParam("LoopDetectorVocabulary")){
    std::string lcd_vocabulary;
    nhp.param<std::string>("LoopDetectorVocabulary", lcd_vocabulary, "");
    ROS_INFO_STREAM("Loop Detector initializing, loading vocabulary");
    try{

      if(descriptor_name.compare("BRIEF") == 0){
        DLDLoopDetector<DBoW2::FBrief>::Parameters lcd_param;
        loop_detector_.reset(new DLDLoopDetector<DBoW2::FBrief>(lcd_vocabulary, lcd_param));
      }else if(descriptor_name.compare("BRISK") == 0){
        DLDLoopDetector<DBoW2::FBRISK>::Parameters lcd_param;
        loop_detector_.reset(new DLDLoopDetector<DBoW2::FBRISK>(lcd_vocabulary, lcd_param));
      }else
        ROS_INFO("Loop Detector could not initialize");

      sptam_->setLoopClosing(loop_detector_);

    } catch (const std::string& exc) {
        ROS_INFO("Loop Detector could not initialize");
        std::cerr << exc << std::endl;
    }
  }
  #endif

  // Subscribe to images messages

  sub_img_l_.subscribe(nhp, "/stereo/left/image_rect", 1);
  sub_info_l_.subscribe(nhp, "/stereo/left/camera_info", 1);
  sub_img_r_.subscribe(nhp, "/stereo/right/image_rect", 1);
  sub_info_r_.subscribe(nhp, "/stereo/right/camera_info", 1);

  if ( use_approx_sync )
  {
    approximate_sync_.reset( new ApproximateSync( ApproximatePolicy(10),
      sub_img_l_, sub_info_l_,
      sub_img_r_, sub_info_r_
    ) );

    approximate_sync_->registerCallback( boost::bind(
      &stereo_driver::onImages, this, _1, _2, _3, _4
    ) );
  }
  else
  {
    exact_sync_.reset( new ExactSync( ExactPolicy(1),
      sub_img_l_, sub_info_l_,
      sub_img_r_, sub_info_r_
    ) );

    exact_sync_->registerCallback( boost::bind(
      &stereo_driver::onImages, this, _1, _2, _3, _4
    ) );
  }

  stereoFrame_Pub_ = imgTransport_.advertise("stereo_frame_before_refine", 100);
  stereoFrameAfter_Pub_ = imgTransport_.advertise("stereo_frame_after_refine", 100);
  leftFrame_Pub_ = imgTransport_.advertise("left_frame_before_refine", 100);
  rightFrame_Pub_ = imgTransport_.advertise("right_frame_before_refine", 100);
  leftFrameAfter_Pub_ = imgTransport_.advertise("left_frame_after_refine", 100);
  rightFrameAfter_Pub_ = imgTransport_.advertise("right_frame_after_refine", 100);

  /*
   * esto permite apagar los threads que usa internamente OpenCV, lo cual en algunas maquinas
   * genera problemas por exceso de threading. Poner 0 hace que no haya paralelismo interno en OpenCV.
   * Se puede usar getNumThreads() para ver cuantos setea por si solo
   */
  //cv::setNumThreads(0);

  ROS_INFO("S-PTAM stereo node initialized.");
}

void sptam::stereo_driver::onImages(
  const sensor_msgs::ImageConstPtr& img_msg_left, const sensor_msgs::CameraInfoConstPtr& left_info,
  const sensor_msgs::ImageConstPtr& img_msg_right, const sensor_msgs::CameraInfoConstPtr& right_info
)
{
  /////////////////////////////
  // Current tate estimation //
  /////////////////////////////

  size_t currentSeq = img_msg_left->header.seq;
  ros::Time currentTime = img_msg_left->header.stamp;
  //ros::Time currentTime = ros::Time::now(); // Gaston: level7_full has a bug on headers stamps (they jump into the future)

  #ifdef SHOW_PROFILING
  writeToLog("FRAME_TIMESTAMP", currentSeq, currentTime.sec, currentTime.nsec);
  #endif

  // Check if a image message was missed
  ROS_DEBUG_STREAM_COND((currentSeq - lastImageSeq_) > 1, "STETEREO FRAME WAS MISSED! current: " << currentSeq << " last: " << lastImageSeq_);
  lastImageSeq_ = currentSeq;

  const CameraPose estimated_camera_pose = estimateCameraPose( currentTime );

  //////////////////////
  // Image processing //
  //////////////////////

  // Extract camera parameters from the first cameraInfo messages.
  if ( not cameraParametersLeft_ )
    loadCameraCalibration(left_info, right_info);

  // convert image to OpenCv cv::Mat format (without modifying color channels)
  cv_bridge::CvImageConstPtr bridgeLeft_ptr = cv_bridge::toCvShare(img_msg_left, "");
  cv_bridge::CvImageConstPtr bridgeRight_ptr = cv_bridge::toCvShare(img_msg_right, "");

  // save images
  cv::Mat imageLeft(bridgeLeft_ptr->image, left_roi_);
  cv::Mat imageRight(bridgeRight_ptr->image, right_roi_);

  #ifdef SHOW_PROFILING
  sptam::Timer t_extraction;
  t_extraction.start();
  #endif
  // Extract features
  FeatureExtractorThread featureExtractorThreadLeft(imageLeft, feature_detector_left_, descriptor_extractor_left_, sptamParameters().nFeatures);
  FeatureExtractorThread featureExtractorThreadRight(imageRight, feature_detector_right_, descriptor_extractor_right_, sptamParameters().nFeatures);

  featureExtractorThreadLeft.WaitUntilFinished();
  const std::vector<cv::KeyPoint>& keyPointsLeft = featureExtractorThreadLeft.GetKeyPoints();
  const cv::Mat& descriptorsLeft = featureExtractorThreadLeft.GetDescriptors();
  featureExtractorThreadRight.WaitUntilFinished();
  const std::vector<cv::KeyPoint>& keyPointsRight = featureExtractorThreadRight.GetKeyPoints();
  const cv::Mat& descriptorsRight = featureExtractorThreadRight.GetDescriptors();

  #ifdef SHOW_PROFILING
  t_extraction.stop();
  WriteToLog(" tk extraction: ", t_extraction);
  sptam::Timer t_hashing;
  t_hashing.start();
  #endif

  ImageFeatures imageFeaturesLeft(cv::Size(left_info->width, left_info->height), keyPointsLeft, descriptorsLeft, sptamParameters().matchingCellSize);
  ImageFeatures imageFeaturesRight(cv::Size(right_info->width, right_info->height), keyPointsRight, descriptorsRight, sptamParameters().matchingCellSize);

  //  std::vector<cv::DMatch> cvMatches;
  //  rowMatcher_->match(keyPointsLeft, descriptorsLeft, keyPointsRight, descriptorsRight, cvMatches);

  //  cv::Mat imageMatches;
  //  cv::drawMatches(imageLeft,keyPointsLeft,imageRight,keyPointsRight,cvMatches, imageMatches);

  //  cv::imshow("Initial Matches",imageMatches);
  //  cv::waitKey(0);

  #ifdef SHOW_PROFILING
  t_hashing.stop();
  WriteToLog(" tk hashing: ", t_hashing);
  WriteToLog(" tk features_left: ", keyPointsLeft.size());
  WriteToLog(" tk features_right: ", keyPointsRight.size());
  #endif

  ///////////////////////
  // Process new frame //
  ///////////////////////

  StereoFrame frame(
    currentSeq,
    estimated_camera_pose, *cameraParametersLeft_,
    stereo_baseline_, *cameraParametersRight_,
    imageFeaturesLeft, imageFeaturesRight, not isMapInitialized()
  );

  sptam::TrackerViewStereo tracker_view(imageLeft, imageRight);

  #ifdef SHOW_TRACKED_FRAMES
  if (stereoFrame_Pub_.getNumSubscribers() > 0)
    tracker_view.enableDrawOutput(TrackerViewStereo::DRAW_BEFORE_REFINE_STEREO);
  else
  {
    if (leftFrame_Pub_.getNumSubscribers() > 0)
      tracker_view.enableDrawOutput(TrackerViewStereo::DRAW_BEFORE_REFINE_LEFT);
    if (rightFrame_Pub_.getNumSubscribers() > 0)
      tracker_view.enableDrawOutput(TrackerViewStereo::DRAW_BEFORE_REFINE_RIGHT);
  }

  if (stereoFrameAfter_Pub_.getNumSubscribers() > 0)
    tracker_view.enableDrawOutput(TrackerViewStereo::DRAW_AFTER_REFINE_STEREO);
  else
  {
    if (leftFrameAfter_Pub_.getNumSubscribers() > 0)
      tracker_view.enableDrawOutput(TrackerViewStereo::DRAW_AFTER_REFINE_LEFT);
    if (rightFrameAfter_Pub_.getNumSubscribers() > 0)
      tracker_view.enableDrawOutput(TrackerViewStereo::DRAW_AFTER_REFINE_RIGHT);
  }
  #endif

  onFrame(currentSeq, currentTime, frame, tracker_view);
}

void sptam::stereo_driver::loadCameraCalibration(
  const sensor_msgs::CameraInfoConstPtr& left_info,
  const sensor_msgs::CameraInfoConstPtr& right_info
)
{
  ROS_INFO_STREAM("init calib");

  // Check if a valid calibration exists
  if (left_info->K[0] == 0.0) {
    ROS_ERROR("The camera is not calibrated");
    return;
  }

  // Ponemos que el frame id de las camara info sea el mismo
  sensor_msgs::CameraInfoPtr left_info_copy = boost::make_shared<sensor_msgs::CameraInfo>(*left_info);
  sensor_msgs::CameraInfoPtr right_info_copy = boost::make_shared<sensor_msgs::CameraInfo>(*right_info);
  left_info_copy->header.frame_id = "stereo";
  right_info_copy->header.frame_id = "stereo";

  // Get Stereo Camera Model from Camera Info message
  image_geometry::StereoCameraModel stereoCameraModel;
  stereoCameraModel.fromCameraInfo(left_info_copy, right_info_copy);

  // Get PinHole Camera Model from the Stereo Camera Model
  const image_geometry::PinholeCameraModel& cameraLeft = stereoCameraModel.left();
  const image_geometry::PinholeCameraModel& cameraRight = stereoCameraModel.right();

  // Get rectify intrinsic Matrix (is the same for both cameras because they are rectify)
  cv::Mat projectionLeft = cv::Mat( cameraLeft.projectionMatrix() );
  cv::Matx33d intrinsicLeft = projectionLeft( cv::Rect(0,0,3,3) );
  cv::Mat projectionRight = cv::Mat( cameraRight.projectionMatrix() );
  cv::Matx33d intrinsicRight = projectionRight( cv::Rect(0,0,3,3) );

  assert(intrinsicLeft == intrinsicRight);

  const cv::Matx33d& intrinsic = intrinsicLeft;

  // Save the baseline
  stereo_baseline_ = stereoCameraModel.baseline();
  ROS_INFO_STREAM("baseline: " << stereo_baseline_);
  assert( stereo_baseline_ > 0 );

  // get the Region Of Interes (If the images are already rectified but invalid pixels appear)
  left_roi_ = cameraLeft.rawRoi();
  right_roi_ = cameraRight.rawRoi();

  cameraParametersLeft_ = std::make_unique<CameraParameters>(cv2eigen(intrinsic), left_roi_.width, left_roi_.height, frustum_near_plane_distance_,  frustum_far_plane_distance_, stereo_baseline_);
  cameraParametersRight_ = std::make_unique<CameraParameters>(cv2eigen(intrinsic), right_roi_.width, right_roi_.height, frustum_near_plane_distance_,  frustum_far_plane_distance_, stereo_baseline_);
}

void sptam::stereo_driver::processTrackerView(const uint32_t seq, const ros::Time& time, const TrackerView& tracker_view_abstract) const
{
  const TrackerViewStereo& tracker_view = static_cast<const TrackerViewStereo&>( tracker_view_abstract );

  cv_bridge::CvImage cv_img;
  cv_img.encoding = "bgr8";
  cv_img.header.seq = seq;
  cv_img.header.stamp = time;

  if (stereoFrame_Pub_.getNumSubscribers() > 0)
  {
    cv_img.image = tracker_view.stereoFrameBeforeRefine;
    stereoFrame_Pub_.publish(cv_img.toImageMsg());
  }

  if (leftFrame_Pub_.getNumSubscribers() > 0)
  {
    cv_img.image = tracker_view.leftFrameBeforeRefine;
    leftFrame_Pub_.publish(cv_img.toImageMsg());
  }

  if (rightFrame_Pub_.getNumSubscribers() > 0)
  {
    cv_img.image = tracker_view.rightFrameBeforeRefine;
    rightFrame_Pub_.publish(cv_img.toImageMsg());
  }

  if (stereoFrameAfter_Pub_.getNumSubscribers() > 0)
  {
    cv_img.image = tracker_view.stereoFrameAfterRefine;
    stereoFrameAfter_Pub_.publish(cv_img.toImageMsg());
  }

  if (leftFrameAfter_Pub_.getNumSubscribers() > 0)
  {
    cv_img.image = tracker_view.leftFrameAfterRefine;
    leftFrameAfter_Pub_.publish(cv_img.toImageMsg());
  }

  if (rightFrameAfter_Pub_.getNumSubscribers() > 0)
  {
    cv_img.image = tracker_view.rightFrameAfterRefine;
    rightFrameAfter_Pub_.publish(cv_img.toImageMsg());
  }
}
