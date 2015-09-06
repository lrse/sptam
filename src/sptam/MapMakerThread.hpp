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

#include "MapMaker.hpp"
#include "utils/concurrent_queue.hpp"

#include <thread>

/**
 * 
 */
class MapMakerThread : public MapMaker
{
  public:

    MapMakerThread(
      sptam::Map& map,
      const CameraParameters& cameraCalibrationLeft,
      const CameraParameters& cameraCalibrationRight,
      const double stereo_baseline,
      const Parameters& params
    );

    /**
     * @override
     */
    void AddKeyFrame(StereoFrame::UniquePtr& keyFrame);

    void Stop();

  private:

  /*** Private properties ***/

    // Queue of keyframes from the tracker waiting to be processed
    /*std::queue<StereoFrame*> keyFrameQueue_;
    // ...
    bool queueEmpty_;
    std::mutex queueMutex_;
    std::condition_variable queueLoaded_;*/
    concurrent_queue<StereoFrame*> keyFrameQueue_;

    // Signal used to break the thread at reasonable points
    bool skipMaintenanceIteration_;

    // Signal used to break the thread when GLOBAL BA IS RUNNING
    bool globalBAIsRunning_;
    
    // Signal to stop the maintenance thread
    bool stop_;

    // Thread running maintainance operations in paralell
    std::thread maintenanceThread_;

  /*** Private functions ***/

    // Mapmaker's code to handle incoming key-frames.
    void Maintenance();
};
