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

#include "tf_utils.hpp"

void waitUtil();


/* This method allows to wait for a transformation between frames that aren't on the current TF Tree.
 * On simulated environment now() won't advance unless the clock is being publish.
 * If there is no one publishing clock, then the waiting will persist */
void waitUntilCanTransform(const tf2_ros::Buffer& tf_buffer,
                           const std::string& targetFrame,
                           const std::string& sourceFrame,
                           const ros::Time& time,
                           const ros::Duration& timeout)
{
  // poll for transform if timeout is set
  ros::Time start_time = ros::Time::now();
  while (ros::Time::now() < start_time + timeout &&
        !tf_buffer.canTransform(targetFrame, sourceFrame, time) &&
        (ros::Time::now()+ros::Duration(3.0) >= start_time) &&  //don't wait when we detect a bag loop
        (ros::ok() || !ros::isInitialized())) // Make sure we haven't been stopped (won't work for pytf)
  {
    ros::Duration(0.01).sleep();
  }
}

void tf2::waitForTransform(const tf2_ros::Buffer& tf_buffer,
                     const std::string& targetFrame,
                     const std::string& sourceFrame,
                     const ros::Time& time,
                     const ros::Duration& timeout,
                     tf2::Transform& sourceToTarget)
{
  // First try to transform the data at the requested time
  try
  {
    tf2::fromMsg(tf_buffer.lookupTransform(targetFrame, sourceFrame, time, timeout).transform, sourceToTarget);
  }
  catch(tf2::LookupException& ex)
  {
    ROS_WARN_STREAM_THROTTLE(2.0, "Frames " << sourceFrame << " or " << targetFrame <<
                                  " doesn't currently exist, waiting and retrying..");

    waitUntilCanTransform(tf_buffer, targetFrame, sourceFrame, time, timeout);
    tf2::fromMsg(tf_buffer.lookupTransform(targetFrame, sourceFrame, time).transform, sourceToTarget);
  }
  catch(tf2::ConnectivityException& ex)
  {
    ROS_WARN_STREAM_THROTTLE(2.0, "Frames " << sourceFrame << " or " << targetFrame <<
                                  " aren't connected on the tf tree, waiting and retrying..");
    waitUntilCanTransform(tf_buffer, targetFrame, sourceFrame, time, timeout);
    tf2::fromMsg(tf_buffer.lookupTransform(targetFrame, sourceFrame, time).transform, sourceToTarget);
  }
  catch (tf2::TransformException& ex)
  {
    // The issue might be that the transforms that are available are not close
    // enough temporally to be used. In that case, just use the latest available
    // transform and warn the user.
    // If this also fails, then just throw an exception.
    geometry_msgs::TransformStamped transformStamped = tf_buffer.lookupTransform(targetFrame, sourceFrame, ros::Time(0));
    tf2::fromMsg(transformStamped.transform, sourceToTarget);

    ROS_WARN_STREAM_THROTTLE(2.0, "Transform from " << sourceFrame << " to " << targetFrame <<
                                  " was unavailable for the time requested (" << time << "). " <<
                                  "Using nearest at time " << transformStamped.header.stamp <<
                                  " instead (" << transformStamped.header.stamp - time << ").");
  }
}

void tf2::lookupTransform(const tf2_ros::Buffer& tf_buffer,
                     const std::string& targetFrame,
                     const std::string& sourceFrame,
                     const ros::Time& time,
                     tf2::Transform& sourceToTarget)
{
  // First try to transform the data at the requested time
  try
  {
    tf2::fromMsg(tf_buffer.lookupTransform(targetFrame, sourceFrame, time).transform, sourceToTarget);
  }
  catch (tf2::TransformException& ex)
  {
    // The issue might be that the transforms that are available are not close
    // enough temporally to be used. In that case, just use the latest available
    // transform and warn the user.
    // If this also fails, then just throw an exception.
    geometry_msgs::TransformStamped transformStamped = tf_buffer.lookupTransform(targetFrame, sourceFrame, ros::Time(0));
    tf2::fromMsg(transformStamped.transform, sourceToTarget);

    ROS_WARN_STREAM_THROTTLE(2.0, "Transform from " << sourceFrame << " to " << targetFrame <<
                                  " was unavailable for the time requested (" << time << "). " <<
                                  "Using nearest at time " << transformStamped.header.stamp <<
                                  " instead (" << transformStamped.header.stamp - time << ").");
  }
}

bool tf2::lookupTransformSafe(const tf2_ros::Buffer& tf_buffer,
                         const std::string& targetFrame,
                         const std::string& sourceFrame,
                         const ros::Time& time,
                         tf2::Transform& sourceToTarget)
{
  bool retVal = true;

  // First try to transform the data at the requested time
  try
  {
    tf2::fromMsg(tf_buffer.lookupTransform(targetFrame, sourceFrame, time).transform, sourceToTarget);
  }
  catch (tf2::TransformException& ex)
  {
    // The issue might be that the transforms that are available are not close
    // enough temporally to be used. In that case, just use the latest available
    // transform and warn the user.
    try
    {
      geometry_msgs::TransformStamped transformStamped = tf_buffer.lookupTransform(targetFrame, sourceFrame, ros::Time(0));
    tf2::fromMsg(transformStamped.transform, sourceToTarget);

    ROS_WARN_STREAM_THROTTLE(2.0, "Transform from " << sourceFrame << " to " << targetFrame <<
                                  " was unavailable for the time requested (" << time << "). " <<
                                  "Using nearest at time " << transformStamped.header.stamp <<
                                  " instead (" << transformStamped.header.stamp - time << ").");
    }
    catch(tf2::TransformException& ex)
    {
      ROS_WARN_STREAM_THROTTLE(2.0, "Could not obtain transform from " << sourceFrame <<
                                    " to " << targetFrame << ". Error was " << ex.what() << "\n");

      retVal = false;
    }
  }

  // Transforming from a frame id to itself can fail when the tf tree isn't
  // being broadcast (e.g., for some bag files). This is the only failure that
  // would throw an exception, so check for this situation before giving up.
  if (not retVal)
  {
    if (targetFrame == sourceFrame)
    {
      sourceToTarget.setIdentity();
      retVal = true;
    }
  }

  return retVal;
}
