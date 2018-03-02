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

#include <vector>
#include <iostream>

// ros::Time
#ifdef BUILDING_ROS
#include <ros/ros.h>
#else
#include "../sptam/utils/time/time.h"
#endif

class Timestamps
{
  public:

    /**
     * @brief create a timestamp generator with a constant rate.
     */
    Timestamps(const double rate, size_t frame_ini = 0);

    /**
     * @brief create a timestamp generator froma a list of timestamps.
     */
    Timestamps(const std::string& filename, size_t frame_ini = 0);

    /**
     * @brief Get the next timestamp when it becomes avaible.
     * This method sleeps for any leftover time since the last time it was called.
     */
    ros::Time getNextWhenReady();

  private:

    bool constant_rate_;

    double rate_;
    ros::Time next_time_;

    size_t next_frame_;
    std::vector<ros::Time> times_;

    ros::Time last_time_;
    ros::Time last_time_update_;
};
