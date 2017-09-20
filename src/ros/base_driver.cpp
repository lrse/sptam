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

#include "base_driver.hpp"

#include "utils/tf_utils.hpp"
#include "utils/tf2eigen.hpp"
#include "utils/opencv_parsers.hpp"
#include "../sptam/MotionModel.hpp"
#include "../sptam/utils/macros.hpp"
#include "../sptam/utils/timer.h"
#include "../sptam/utils/log/Profiler.hpp"

#include <nav_msgs/Path.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseStamped.h>

#define INITIAL_COVARIANCE ( Eigen::Matrix6d::Identity() * 1e-9 )

// Convert from CameraPose to tf2::Pose (position and orientation)
inline void CameraPose2TFPose(const CameraPose& cameraPose, tf2::Transform& pose)
{
  const Eigen::Quaterniond& orientation = cameraPose.GetOrientationQuaternion();
  const Eigen::Vector3d&  position = cameraPose.GetPosition();
  pose.setOrigin(tf2::Vector3(position[0], position[1], position[2]));
  pose.setRotation(tf2::Quaternion( orientation.x(), orientation.y(), orientation.z(), orientation.w() )); // q = (x,y,z,w)
}

// Convert from tf2::Pose to CameraPose (position and orientation)
inline CameraPose TFPose2CameraPose(const tf2::Transform& pose)
{
  // convert to position opencv vector
  tf2::Vector3 position_tf = pose.getOrigin();
  Eigen::Vector3d position(position_tf.getX(), position_tf.getY(), position_tf.getZ());

  // Convert to orientation opencv quaternion
  tf2::Quaternion orientation_tf = pose.getRotation();
  Eigen::Quaterniond orientation(orientation_tf.getW(), orientation_tf.getX(), orientation_tf.getY(), orientation_tf.getZ());

  // TODO is it OK to use the Id covariance here?
  return CameraPose(position, orientation, Eigen::Matrix6d::Identity());
}

// ================================================================== //

sptam::base_driver::base_driver(ros::NodeHandle& nh, ros::NodeHandle& nhp)
  : sptam_( nullptr )
  , pose_predictor_( nullptr )
  , cameraPose_(Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), INITIAL_COVARIANCE)
  , published_transform_( tf2::Transform::getIdentity() )
  , transform_listener_( tfBuffer_ )
{
  #if CV_MAJOR_VERSION == 2
    // Load Nonfree OpenCv modules (SURF,SIFT)
    cv::initModule_nonfree();
  #endif // CV_MAJOR_VERSION

  // Get node parameters

  nhp.param<bool>("use_prediction", use_prediction_, false);
  nhp.param<bool>("publish_transform", publish_transform_, true);
  nhp.param<bool>("publish_on_fail", publish_on_fail_, false);

  nhp.param<std::string>("prediction_frame", prediction_frame_, "odom");
  nhp.param<std::string>("base_frame", base_frame_, "base_link");
  nhp.param<std::string>("camera_frame", camera_frame_, "camera");
  nhp.param<std::string>("map_frame", map_frame_, "map");
  nhp.param<std::string>("reference_frame", reference_frame_, "base_link");


  // TODO: Warning the nFeatures parameter may not appear or have different name in OpenCV and therefore it could be not set.
  // For Example: GFTT uses nfeatures (low capital letter f) instead of nFeatures!!!!
  nhp.getParam("FeatureDetector/nFeatures", sptam_params_.nFeatures);

  // load descriptor matcher
  {
    std::string matcherName;
    nhp.param<std::string>("DescriptorMatcher/Name", matcherName, "BruteForce-Hamming");

    sptam_params_.descriptorMatcher = loadDescriptorMatcher(nhp, matcherName, "DescriptorMatcher" );
  }

  // Load Parameters
  // nhp.param is not overloaded for unsigned int
  nhp.getParam("MatchingCellSize", sptam_params_.matchingCellSize);
  nhp.getParam("MatchingDistance", sptam_params_.matchingDistanceThreshold);
  nhp.getParam("MatchingNeighborhood", sptam_params_.matchingNeighborhoodThreshold);
  nhp.getParam("EpipolarDistance", sptam_params_.epipolarDistanceThreshold);

  // BA parameters
  nhp.getParam("BundleAdjustmentActiveKeyframes", sptam_params_.nKeyFramesToAdjustByLocal);
  nhp.getParam("maxIterationsLocal", sptam_params_.maxIterationsLocal);

  // Keyframe creation policy parameters
  nhp.getParam("minimumTrackedPointsRatio", sptam_params_.minimumTrackedPointsRatio);

  // write configuration to log file
  #ifdef SHOW_PROFILING
    Logger::Write( "#   matchingCellSize: " + std::to_string(sptam_params_.matchingCellSize) + "\n" );
    Logger::Write( "#   matchingNeighborhoodThreshold: " + std::to_string(sptam_params_.matchingNeighborhoodThreshold) + "\n" );
    Logger::Write( "#   matchingDistanceThreshold: " + std::to_string(sptam_params_.matchingDistanceThreshold) + "\n" );
    Logger::Write( "#   epipolarDistanceThreshold: " + std::to_string(sptam_params_.epipolarDistanceThreshold) + "\n" );
    Logger::Write( "#   BundleAdjustmentActiveKeyframes: " + std::to_string( sptam_params_.nKeyFramesToAdjustByLocal ) + "\n" );
  #endif

  // Create RowMatcher instance
  rowMatcher_ = std::make_unique<RowMatcher>( sptam_params_.matchingDistanceThreshold, sptam_params_.descriptorMatcher, sptam_params_.epipolarDistanceThreshold );

  mapPub_ = nhp.advertise<sensor_msgs::PointCloud2>("global_map", 100);
  localMapPub_ = nhp.advertise<sensor_msgs::PointCloud2>("local_map", 100);
  trackedMapPub_ = nhp.advertise<sensor_msgs::PointCloud2>("tracked_map", 100);
  posePub_ = nhp.advertise<geometry_msgs::PoseWithCovarianceStamped>("robot/pose", 100);
  keyframesPub_ = nhp.advertise<nav_msgs::Path>("keyframes", 100);
  localKeyframesPub_ = nhp.advertise<nav_msgs::Path>("local_keyframes", 100);

  published_frame_ = use_prediction_ ? prediction_frame_ : base_frame_;

  // Create SPTAM instance. The reason we can't initialize it
  // in the initialization list is because sptam parameters are loaded
  // in the constructor body.
  sptam_ = std::make_unique<SPTAM>(*rowMatcher_, sptam_params_);

  // We can't to do anything ROS related in the constructor. Because of this,
  // we initialize a 0s timer which calls an initialization function once
  // during the first ros spin.
  initialization_timer_ = nhp.createTimer(ros::Duration(0), &base_driver::initialize, this, true);
}

sptam::base_driver::~base_driver()
{
  // we use couts because ROS is dying
  std::cout << "starting sptam node cleanup..." << std::endl;

  std::cout << "stopping sptam threads..." << std::endl;
  sptam_->stop();

  #ifdef SHOW_PROFILING
    std::cout << "saving measurements..." << std::endl;
    for ( const auto& mapPoint : sptam_->GetMap().getMapPoints() ) {
      WriteToLog( " tk MeasurementCount: ", mapPoint->measurements().size() );
    }
  #endif

  #ifdef SHOW_PROFILING
    std::cout << "saving keyframes..." << std::endl;
    for ( const auto& keyFrame : sptam_->GetMap().getKeyframes() ) {
      const CameraPose keyFramePose = keyFrame->GetCameraPose();
      
      /* Applying frame transformations to match pose information with
       * those reported on every tracked frame */
      tf2::Transform mapcam_from_camera;
      CameraPose2TFPose( keyFramePose, mapcam_from_camera );
      tf2::Transform map_from_base = map_from_mapcam_ * mapcam_from_camera * camera_from_base_;
      
      writePoseToLog("FINAL_FRAME_POSE", keyFrame->GetId(), tf2eigen( map_from_base.getOrigin() ), tf2eigen( map_from_base.getBasis() ), keyFramePose.covariance());
    }
  #endif

  // create map file
  {
    std::cout << "saving map cloud..." << std::endl;
    std::ofstream out("map cloud.dat");
    for ( const auto& point : sptam_->GetMap().getMapPoints() )
      out << point->GetPosition().x() << " " << point->GetPosition().y() <<  " " << point->GetPosition().z() << std::endl;
    out.close();
  }

  std::cout << "done!" << std::endl;

  // sleep for one second
  //ros::Duration( 1.0 ).sleep();
}

void sptam::base_driver::onFrame(const uint32_t seq, const ros::Time& time, StereoFrame& frame, TrackerView& tracker_view)
{
  bool publish_pose = true;

  // if the map was not initialized, try to build it
  if( not sptam_->isInitialized() )
  {
    ROS_INFO_THROTTLE(1, "Trying to intialize map...");

    try {
      sptam_->init( frame );
    }
    catch( std::runtime_error& err ) {
      ROS_WARN_STREAM_THROTTLE(1, err.what());
      return;
    }

    const CameraPose& camera_pose = frame.GetCameraPose();
    pose_predictor_ = std::shared_ptr<MotionModel>(new MotionModel(time, camera_pose.GetPosition(), camera_pose.GetOrientationQuaternion(), camera_pose.covariance()));

    ROS_INFO_STREAM("Map initialized with " << sptam_->GetMap().getMapPoints().size() << " points.");
  }
  // if the map is already initialized, do tracking
  else
  {
    #ifdef SHOW_PROFILING
    const CameraPose& camera_pose = frame.GetCameraPose();
    writePoseToLog("ESTIMATED_CAMERA_POSE", seq, camera_pose.GetPosition(), camera_pose.GetOrientationMatrix(), camera_pose.covariance());
    #endif

    #ifdef SHOW_PROFILING
    sptam::Timer t_total;
    t_total.start();
    #endif // SHOW_PROFILING

    TrackingReport tracking_report = sptam_->track(frame, tracker_view);

    #ifdef SHOW_PROFILING
      t_total.stop();
      WriteToLog(" tk trackingtotal: ", t_total);
    #endif

    if ( not tracking_report.isOk() )
      ROS_WARN_STREAM("Not enough points for tracking.");

    const CameraPose refined_camera_pose = tracking_report.refinedCameraPose;

    #ifdef SHOW_PROFILING
      writePoseToLog("REFINED_CAMERA_POSE", seq, refined_camera_pose.GetPosition(), refined_camera_pose.GetOrientationMatrix(), refined_camera_pose.covariance() );
    #endif
    
    checkTrajectoryCorrection(frame, tracking_report);

    publish_pose = tracking_report.isOk() or publish_on_fail_;

    if( publish_pose and publish_transform_ )
      updatePublishedTransform(time, refined_camera_pose);

    if ( tracking_report.isOk() )
      updateCameraPose(time, refined_camera_pose);

    // publish processed camera frames with debug information
    processTrackerView(seq, time, tracker_view);

    publishPoints(seq, time, tracking_report.localMap, localMapPub_);
    publishPoints(seq, time, tracking_report.trackedMap, trackedMapPub_);
    publishKFs(seq, time, tracking_report.localKeyFrames, localKeyframesPub_);
  }

  // Publish Map To be drawn by rviz visualizer
  if (mapPub_.getNumSubscribers() > 0)
  {
    boost::shared_lock<boost::shared_mutex> lock(sptam_->GetMap().map_mutex_);
    publishPoints(seq, time, sptam_->GetMap().getMapPoints(), mapPub_);
  }

  if (keyframesPub_.getNumSubscribers() > 0)
  {
    boost::shared_lock<boost::shared_mutex> lock(sptam_->GetMap().map_mutex_);
    publishKFs(seq, time, sptam_->GetMap().getKeyframes(), keyframesPub_);
  }

  // Publish the camera Pose
  if ( publish_pose )
    publishPose(seq, time, cameraPose_);

  if ( publish_pose and publish_transform_ )
    publishTransform(time, published_transform_);
}

void sptam::base_driver::initialize(const ros::TimerEvent& event)
{
  // Compute transform between the internal and published map reference frames
  // and save it in map_from_mapcam_.
  tf2::waitForTransform(tfBuffer_, reference_frame_, camera_frame_, ros::Time::now(), ros::Duration(3), map_from_mapcam_);
}

bool sptam::base_driver::getBaseLinkPose(const CameraPose& cameraPose, const ros::Time& t, tf2::Transform& map_from_base)
{
  if (not lookupTransformSafe(tfBuffer_, camera_frame_, base_frame_, t, camera_from_base_))
    return false;

  tf2::Transform mapcam_from_camera;
  CameraPose2TFPose( cameraPose, mapcam_from_camera );

  map_from_base = map_from_mapcam_ * mapcam_from_camera * camera_from_base_;

  return true;
}

template <class T>
void sptam::base_driver::publishPoints(const uint32_t seq, const ros::Time& time, const T& points, ros::Publisher& pub)
{
  if ( pub.getNumSubscribers() < 1 )
    return;
    
  // Create PointCloud message for visualization
  pcl::PointCloud<pcl::PointXYZRGB> msg;
  msg.header.seq = seq;
  msg.header.frame_id = map_frame_;
  msg.height = 1;
  msg.width = points.size();
  for(const auto& mapPoint : points )
  {
    // Get Point from Map
    const Eigen::Vector3d point3d = mapPoint->GetPosition();

    // Transform to the correct reference frame
    const tf2::Vector3 point_tf2_local(point3d.x(), point3d.y(), point3d.z());
    const tf2::Vector3 point_tf2 = map_from_mapcam_ * point_tf2_local;

    // Build PCL point
    pcl::PointXYZRGB point_pcl;

    point_pcl.x = point_tf2.x();
    point_pcl.y = point_tf2.y();
    point_pcl.z = point_tf2.z();

    cv::Vec3b color = mapPoint->getColor();

    point_pcl.r = color(0);
    point_pcl.g = color(1);
    point_pcl.b = color(2);

    msg.points.push_back ( point_pcl );
  }

  // Publish the PointCloud
  pub.publish( msg );
}

void sptam::base_driver::publishPose(const uint32_t seq, const ros::Time& time, const CameraPose& currentCameraPose)
{
  tf2::Transform map_from_base;
  if ( not getBaseLinkPose( currentCameraPose, time, map_from_base ) )
  {
    ROS_WARN("Failed to retrieve camera pose in base_link frame. Ignoring pose publication.");
    return;
  }

  // TODO this is the covariance of the camera_frame_, shouldn't
  // it be translated to the base_frame_?
  const Eigen::Matrix6d& covariance = currentCameraPose.covariance();

  #ifdef SHOW_PROFILING
    writePoseToLog("TRACKED_FRAME_POSE", seq, tf2eigen( map_from_base.getOrigin() ), tf2eigen( map_from_base.getBasis() ), covariance);
  #endif

  geometry_msgs::PoseWithCovarianceStamped msg;

  msg.header.seq = seq;
  msg.header.stamp = time;
  msg.header.frame_id = map_frame_;

  msg.pose.pose.orientation.x = map_from_base.getRotation().x();
  msg.pose.pose.orientation.y = map_from_base.getRotation().y();
  msg.pose.pose.orientation.z = map_from_base.getRotation().z();
  msg.pose.pose.orientation.w = map_from_base.getRotation().w();

  msg.pose.pose.position.x = map_from_base.getOrigin().x();
  msg.pose.pose.position.y = map_from_base.getOrigin().y();
  msg.pose.pose.position.z = map_from_base.getOrigin().z();

  // Row-major representation of the 6x6 covariance matrix
  // The orientation parameters use a fixed-axis representation.
  // In order, the parameters are:
  // (x, y, z, rotation about X axis, rotation about Y axis, rotation about Z axis)
  forn(i, 6) forn(j, 6)
    msg.pose.covariance[6*i + j] = covariance(i, j);

  // Publish the camera pose
  posePub_.publish( msg );
}

template<class T>
void sptam::base_driver::publishKFs(const uint32_t seq, const ros::Time& time, const T& keyframes, ros::Publisher& pub)
{
  if ( pub.getNumSubscribers() < 1 )
    return;

  nav_msgs::Path keyframes_msg;
  keyframes_msg.header.seq = seq;
  keyframes_msg.header.stamp = time;
  keyframes_msg.header.frame_id = map_frame_;

  for(const auto& keyframe : keyframes){
    geometry_msgs::PoseStamped pose_msg;

    pose_msg.header.seq = seq;
    pose_msg.header.stamp = time;
    pose_msg.header.frame_id = map_frame_;
    
    tf2::Transform kf_pose;
    CameraPose2TFPose( keyframe->GetCameraPose(), kf_pose );

    kf_pose = map_from_mapcam_ * kf_pose; /* transform from mapcam to map */
    
    pose_msg.pose.orientation.x = kf_pose.getRotation().x();
    pose_msg.pose.orientation.y = kf_pose.getRotation().y();
    pose_msg.pose.orientation.z = kf_pose.getRotation().z();
    pose_msg.pose.orientation.w = kf_pose.getRotation().w();

    pose_msg.pose.position.x = kf_pose.getOrigin().x();
    pose_msg.pose.position.y = kf_pose.getOrigin().y();
    pose_msg.pose.position.z = kf_pose.getOrigin().z();

    keyframes_msg.poses.push_back(pose_msg);
  }

  // Publish the camera pose
  pub.publish( keyframes_msg );
}

void sptam::base_driver::publishTransform(const ros::Time& time, const tf2::Transform& published_transform)
{
  ros::Time tf_expiration = time;

  geometry_msgs::TransformStamped published_transform_msg;
  published_transform_msg.header.frame_id = map_frame_;
  published_transform_msg.header.stamp = tf_expiration;
  published_transform_msg.child_frame_id = published_frame_;

  published_transform_msg.transform = tf2::toMsg( published_transform );

  transform_broadcaster_.sendTransform( published_transform_msg );
}

CameraPose sptam::base_driver::cameraPoseFromPredictor(const ros::Time& time) const
{
  Eigen::Vector3d currentCameraPosition;
  Eigen::Quaterniond currentCameraOrientation;
  Eigen::Matrix6d predictionCovariance;

  pose_predictor_->predictPose(time, currentCameraPosition, currentCameraOrientation, predictionCovariance);

  return CameraPose(currentCameraPosition, currentCameraOrientation, predictionCovariance);
}

CameraPose sptam::base_driver::estimateCameraPose(const ros::Time& time) const
{
  // If external prediction is enabled, try to get a new pose estimate from the tf tree.
  if ( use_prediction_ )
  {
    tf2::Transform prediction_from_camera;
    if ( lookupTransformSafe(tfBuffer_, prediction_frame_, camera_frame_, time, prediction_from_camera) )
    {
      // When external prediction is enabled, published_transform_ contains the map_from_prediction transformation.
      tf2::Transform map_from_camera = published_transform_ * prediction_from_camera;
      tf2::Transform mapcam_from_camera = map_from_mapcam_.inverse() * map_from_camera;
      return TFPose2CameraPose( mapcam_from_camera );
    }
    // if prediction fails, return last known camera pose ( cameraPose_ ).
    else {
      ROS_WARN_STREAM_THROTTLE(1, "Could not get prediction " << map_frame_ << "->" << camera_frame_ << ". Using last known pose instead.");
      return cameraPose_;
    }
  }
  // If external prediction is disabled, try to get a new pose estimate from the motion model.
  else
  {
    if ( not pose_predictor_ )
      return cameraPose_;

    return cameraPoseFromPredictor( time );
  }
}

void sptam::base_driver::checkTrajectoryCorrection(const StereoFrame& estimatedFrame, const TrackingReport& report)
{
  // PosePredictor needs to be notify of this abrupt correction
  if ( use_prediction_ || report.T_corr.isIdentity()) // nothing to correct
    return;
  
  if(pose_predictor_ != nullptr)
    pose_predictor_->applyCorrection(report.T_corr);  
  
  #ifdef SHOW_PROFILING
    writeTransfToLog("ESTIMATION_CORRECTION: ", estimatedFrame.GetId(), report.T_corr);
  #endif
}

void sptam::base_driver::updateCameraPose(const ros::Time& time, const CameraPose& new_camera_pose)
{
  cameraPose_ = new_camera_pose;

  // The "update" will be done on TF by updatePublishedTransform
  if ( use_prediction_ )
    return;

  pose_predictor_->updatePose(time, new_camera_pose.GetPosition(), new_camera_pose.GetOrientationQuaternion(), new_camera_pose.covariance());

  // The pose computed by the pose predictor may not be the same
  // as the given update. One example is when we are using EKF.
  cameraPose_ = cameraPoseFromPredictor( time );
}

void sptam::base_driver::updatePublishedTransform(const ros::Time& time, const CameraPose& cameraPose)
{
  tf2::Transform camera_from_published;
  if ( lookupTransformSafe(tfBuffer_, camera_frame_, published_frame_, time, camera_from_published) )
  {
    tf2::Transform mapcam_from_camera;
    CameraPose2TFPose(cameraPose, mapcam_from_camera);

    published_transform_ = map_from_mapcam_ * mapcam_from_camera * camera_from_published;
  }
  else
    ROS_WARN_STREAM_THROTTLE(1, "Could not update " << map_frame_ << "->" << published_frame_ << " transform, because " << camera_frame_ << "->" << published_frame_ << " transform was not found.");
}
