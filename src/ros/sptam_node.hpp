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
#pragma once

#include "../sptam/sptam.hpp"
#include "../sptam/MotionModel.hpp"

#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>

namespace sptam
{

class sptam_node
{

  public:

    sptam_node(ros::NodeHandle& nh, ros::NodeHandle& nhp);

    ~sptam_node();

  private:

    // Compute baseline
    void loadCameraCalibration( const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                                const sensor_msgs::CameraInfoConstPtr& r_info_msg );

    /**
     * @brief
     *   Syncronized image callback. Tracks the features on the new images
     *   to compute the current robot position.
     */
    void onImages( const sensor_msgs::ImageConstPtr& l_image_msg,
                   const sensor_msgs::CameraInfoConstPtr& l_info_msg,
                   const sensor_msgs::ImageConstPtr& r_image_msg,
                   const sensor_msgs::CameraInfoConstPtr& r_info_msg );

    /**
     * @brief
     *   Get the current camera pose at time t in the map reference frame.
     */
    bool getCameraInOdom(tf::StampedTransform& camera_to_odom, const ros::Time& t);

    bool getBaseLinkPose(const CameraPose& cameraPose, const ros::Time& t, tf::Pose& base_to_map);

    /**
     * @brief
     *   fix the error between map and odom to correct the current pose
     *   by publishing a corrected tf from map to odom frame.
     *
     * @param camera_pose
     *   current (refined) camera pose in the map reference frame
     */
    void fixOdomFrame(const CameraPose& cameraPose, const tf::StampedTransform& camera_to_odom, const ros::Time& t);

    void publishMap();
    void publishPath();
    void publishPose(const uint32_t seq, const ros::Time& time, const CameraPose& currentCameraPose);

    void publishTransform();
    void publishTransformLoop();

    // TODO change names, maybe unify publishTransformLoop
    // 1 and 2 using a function parameter.
    void publishTransform2();
    void publishTransformLoop2();

  // Method variables

    Map map_;

    MapMaker::Parameters mapper_params_;
    cv::Ptr<cv::FeatureDetector> featureDetector_;
    cv::Ptr<cv::DescriptorExtractor> descriptorExtractor_;
    RowMatcher* rowMatcher_;

    SPTAM* sptam_;

    CameraParameters cameraParametersLeft_;
    CameraParameters cameraParametersRight_;

    double stereo_baseline_;

    // used to save the last computed camera pose, in the map
    // coordinate frame, when odometry is disabled.
    CameraPose cameraPose_;
    std::unique_ptr<MotionModel> motionModel_;

  // node parameters

    bool use_odometry_;
    double tf_delay_, transform_publish_freq_;
    std::string odom_frame_, base_frame_, camera_frame_, map_frame_;

  // ROS variables

    ros::Publisher mapPub_, posePub_;

    // Publish extracted image features for debugging purposes
    image_transport::ImageTransport it_;
    image_transport::Publisher leftKpPub_, rightKpPub_;

    /**
     * Keep a local cache of current map->odom transformation, because
     * this node only publishes it when a correction is necessary and
     * the time between those corrections may be too large for tf to
     * when trying to use the last published value.
     */
    tf::Transform odom_to_map_;
    std::mutex odom_to_map_mutex_;
    std::thread* transform_thread_;

    tf::TransformListener transform_listener_;
    tf::TransformBroadcaster transform_broadcaster_;

    // Subscribers for the image topics
    message_filters::Subscriber<sensor_msgs::Image> sub_l_image_, sub_r_image_;
    message_filters::Subscriber<sensor_msgs::CameraInfo> sub_l_info_, sub_r_info_;

    // syncronizer for image messages. We don't know a priory which one
    // will be used, so we have to define both, no common interface :/

    // Exact time image topic synchronizer
    typedef message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo> ExactPolicy;
    typedef message_filters::Synchronizer<ExactPolicy> ExactSync;
    boost::shared_ptr<ExactSync> exact_sync_;

    // Approximate time image topic synchronizer
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image, sensor_msgs::CameraInfo> ApproximatePolicy;
    typedef message_filters::Synchronizer<ApproximatePolicy> ApproximateSync;
    boost::shared_ptr<ApproximateSync> approximate_sync_;

}; // class sptam

} // namespace sptam
