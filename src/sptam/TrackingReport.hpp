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

#include "CameraPose.hpp"
#include "Map.hpp"

/**
 * @brief Holds information regarding the result of a tracking operation
 */
class TrackingReport
{
  public:

    TrackingReport()
      : T_corr(Eigen::Matrix4d::Identity()), state( State::OK )
    {}

    CameraPose refinedCameraPose;

    /* Correction applied to the inputed estimatedCameraPose before refinement.
     * This corresponds to a loop correction applied to the ongoing trajectory */
    Eigen::Matrix4d T_corr;

    sptam::Map::SharedMapPointList localMap;
    sptam::Map::SharedMapPointSet trackedMap;
    sptam::Map::SharedKeyFrameSet localKeyFrames;

    /**
     * Describes the possible outcomes of tracking
     */
    enum class State {
      OK,                  /** tracking was succesful */
      NOT_ENOUGH_POINTS    /** there were not enough points for tracking */
    };
    State state;

    inline bool isOk() const
    { return state == TrackingReport::State::OK; }
};
