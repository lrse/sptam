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

#include <thread>
#include <list>
#include <tuple>

// Eigen
#include <Eigen/Core>

#include "LCDetector.hpp"
#include "../Map.hpp"
#include "../CameraParameters.hpp"
#include "../utils/concurrent_queue.hpp"
#include "../utils/eigen_alignment.hpp"

// Forward declarations due to cross reference between sptam, loopclosing and mapmaker
class SPTAM;
class MapMakerThread;

/**
 * 
 */
class LoopClosing
{
  public:
    /**
     * Collection of some tuning parameters
     * that are given to the LoopClosure
     */
    struct Parameters
    {
      cv::Ptr<cv::DescriptorMatcher> descriptorMatcher;
      double matchingDistanceThreshold = 25.0;
    };
    
    LoopClosing(SPTAM& tracker, MapMakerThread& mapper, sptam::Map& map, std::unique_ptr<LCDetector>& detector, const Parameters& params);
    ~LoopClosing();
    
    void addKeyFrame(sptam::Map::SharedKeyFrame& keyFrame);

    void addKeyFrames(const std::list< sptam::Map::SharedKeyFrame >& keyFrames);

    inline void stop()
    { stop_ = true; keyFrameQueue_.stop(); maintenanceThread_.join(); }
  
  private:
  
    SPTAM& tracker_;
    MapMakerThread& mapper_;
    sptam::Map& map_;
    std::unique_ptr<LCDetector> loop_detector_; // LoopClosing is on charge of the detector!

    Parameters params_;

    std::aligned_vector<std::tuple<size_t, size_t, Eigen::Isometry3d>> loop_frames_;

    concurrent_queue<sptam::Map::SharedKeyFrame> keyFrameQueue_;
    
    // Signal to stop the maintenance thread
    bool stop_ = false;

    // Thread running maintainance operations in paralell
    std::thread maintenanceThread_;
    
    void maintenance();
};
