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

#include "Match.hpp"
#include "CameraPose.hpp"

namespace Eigen
{
  typedef Matrix<double, 6, 6> Matrix6d;
}

class tracker_g2o
{
  public:

    /**
     * @brief This should be called for every frame of incoming stereo images.
     * This will use an estimate of the current camera pose and will try
     * to adjust it to the current map by minimizing reprojection errors.
     */
    CameraPose RefineCameraPose(
      const CameraPose& estimatedCameraPose,
      const sptam::RectifiedCameraParameters& rectified_camera_parameters,
      const std::list<Match>& measurements
    );

    class not_enough_points : public std::runtime_error
    {
      public:
        not_enough_points()
          : std::runtime_error("Not enough points for tracking.") {}
    };
};
