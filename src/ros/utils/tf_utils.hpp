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

#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

namespace tf2
{

void waitForTransform(const tf2_ros::Buffer& tf_buffer,
                     const std::string& targetFrame,
                     const std::string& sourceFrame,
                     const ros::Time& time,
                     const ros::Duration& timeout,
                     tf2::Transform& sourceToTarget);

void lookupTransform(const tf2_ros::Buffer& tf_buffer,
                     const std::string& targetFrame,
                     const std::string& sourceFrame,
                     const ros::Time& time,
                     tf2::Transform& sourceToTarget);

bool lookupTransformSafe(const tf2_ros::Buffer& tf_buffer,
                         const std::string& targetFrame,
                         const std::string& sourceFrame,
                         const ros::Time& time,
                         tf2::Transform& sourceToTarget);
 
} // tf2
