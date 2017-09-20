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
#pragma once

#include "../sptam/sptam.hpp"
#include "../sptam/PosePredictor.hpp"

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <tf2/LinearMath/Transform.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

namespace sptam
{

class base_driver
{
  public:

    base_driver(ros::NodeHandle& nh, ros::NodeHandle& nhp);

    virtual ~base_driver();

  protected:
  
    // Protected instances, as inherited classes may need to modify them
    std::unique_ptr<SPTAM> sptam_;    
    std::shared_ptr<PosePredictor> pose_predictor_;

    void onFrame(const uint32_t seq, const ros::Time& time, StereoFrame& frame, TrackerView& tracking_report);

    CameraPose estimateCameraPose(const ros::Time& time) const;

    virtual void processTrackerView(const uint32_t seq, const ros::Time& time, const TrackerView& report) const {};

    const Parameters& sptamParameters() const
    { return sptam_params_; }

    // little hack to mark the frame as static from the outside
    bool isMapInitialized() const
    { return sptam_->isInitialized(); }

  private:
  
    void checkTrajectoryCorrection(const StereoFrame& estimatedFrame, const TrackingReport& report);

    void updateCameraPose(const ros::Time& time, const CameraPose& new_camera_pose);

    void initialize(const ros::TimerEvent& event);

    CameraPose cameraPoseFromPredictor(const ros::Time& time) const;

    bool getBaseLinkPose(const CameraPose& cameraPose, const ros::Time& t, tf2::Transform& base_to_map);

    void publishPose(const uint32_t seq, const ros::Time& time, const CameraPose& currentCameraPose);

    void publishTransform(const ros::Time& time, const tf2::Transform& published_transform);

    void updatePublishedTransform(const ros::Time& time, const CameraPose& cameraPose);

    template<class T>
    void publishKFs(const uint32_t seq, const ros::Time& time, const T& keyframes, ros::Publisher& pub);

    template<class T>
    void publishPoints(const uint32_t seq, const ros::Time& time, const T& points, ros::Publisher& pub);

  // Method variables

    // We can't to do anything ROS related in the constructor. Because of this,
    // we initialize a 0s timer which calls an initialization function once
    // during the first ros spin.
    ros::Timer initialization_timer_;

    std::unique_ptr<RowMatcher> rowMatcher_;

    Parameters sptam_params_;

    // used to save the last computed camera pose, in the mapcam
    // coordinate frame.
    mutable CameraPose cameraPose_;

  // node parameters

    bool use_prediction_, publish_transform_, publish_on_fail_;
    std::string prediction_frame_, base_frame_, camera_frame_, map_frame_, reference_frame_;

    /**
     * If enabled, the node will publish the transformation
     * map_frame_->published_frame_ to tf.
     * If using external prediction this will be the prediction_frame_ frame
     * and if not, then it will be the base_frame_.
     */
    std::string published_frame_;

  // ROS variables

    ros::Publisher mapPub_, localMapPub_, trackedMapPub_, keyframesPub_, localKeyframesPub_, posePub_;

    /**
     * This transform contains the map_frame_->prediction_frame_ transform when
     * external prediction is enabled, and the map_frame_->base_frame_
     * transform when not.
     *
     * The reason we need to save it as a local property is the following:
     * The transform is published only when updated, that is after each
     * succesfull tracking operation. This means that when we ask for a new
     * pose prediction for the frame 'k+1', when using external prediction,
     * the information is read from the tf tree, and the last complete tranform
     * from map_frame_->camera_frame_ corresponds to (at least) the last time
     * the map_frame_->prediction_frame_ transform was published. That is,
     * the transform we published at the time of frame 'k'.
     * This means the whole prediction information gathered between the frames,
     * which is actually the reason for using a prediction, is not used at all.
     * That is why we save this information locally and ask only for the
     * prediction_frame_->base_frame_ transform, and combine the transformation
     * internally.
     */
    tf2::Transform published_transform_;

    /**
     * The map computed by S-PTAM has it's coordinate origin aligned with the
     * camera frame at the moment the Map is initialized.
     * Since this is not very intuitive, we give an option to define a
     * 'reference frame', by default base_link, which will be the original
     * reference frame for the published map frame, instead of the camera frame.
     * This transformation between the internal map, and the
     * published map reference frames is given by map_from_mapcam_.
     */
    tf2::Transform map_from_mapcam_;
    
    
    /**
     * Transformation between robot's base_link and the mounted camera. This
     * transformation may change overtime but is stored so it can be used
     * on node destruction to print keyframe poses relative to required frames.
     */
    tf2::Transform camera_from_base_;

    tf2_ros::Buffer tfBuffer_;
    tf2_ros::TransformListener transform_listener_;
    tf2_ros::TransformBroadcaster transform_broadcaster_;
};

} // sptam
