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
#include "StereoImageFeatures.hpp"

class SptamWrapper
{
  public:

    SptamWrapper(
      const CameraParameters& cameraParametersLeft,
      const CameraParameters& cameraParametersRight,
      const double stereo_baseline,
      const RowMatcher& rowMatcher,
      const Parameters& params,
      std::shared_ptr<PosePredictor> motionModel,
      const size_t imageBeginIndex
    );

    #ifdef USE_LOOPCLOSURE
    void setLoopClosing(std::unique_ptr<LCDetector>& loop_detector);
    #endif

    virtual void Add(const size_t frame_id, const ros::Time& time, std::unique_ptr<StereoImageFeatures> stereoImageFeatures);

    inline sptam::Map& GetMap()
    { return sptam_.GetMap(); }

    virtual void Stop()
    { sptam_.stop(); }

  protected:

    const CameraParameters cameraParametersLeft_;
    const CameraParameters cameraParametersRight_;
    const double stereo_baseline_;
    RowMatcher rowMatcher_;
    Parameters params_;
    std::shared_ptr<PosePredictor> motionModel_;
    bool isMapInitialized_;
    SPTAM sptam_;
};
