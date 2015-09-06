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

#include "Map.hpp"
#include "tracker_g2o.hpp"
#include "MapMakerThread.hpp"

class SPTAM
{
  public:

    SPTAM(
      sptam::Map& map,
      const CameraParameters& cameraParametersLeft,
      const CameraParameters& cameraParametersRight,
      const double stereo_baseline,
      const RowMatcher& rowMatcher,
      const MapMaker::Parameters& params
    );

    CameraPose track(
      const CameraPose& estimatedCameraPose,
      const ImageFeatures &imageFeaturesLeft,
      const ImageFeatures &imageFeaturesRight,
      const cv::Mat& imageLeft,
      const cv::Mat& imageRight);

    inline void stop()
    { mapMaker_.Stop(); }

  protected:

    // TODO: feo, no deberíamos hacer una copia?
    // the map (points + views)
    sptam::Map& map_;

    // the map builder interface
    // To run the mapper in a parallel thread, build an instance
    // of MapMakerThread.
    // If the run should be sequential, build an instance of MapMaker.
    MapMakerThread mapMaker_;

    // the tracker interface
    // To run the the tracker with g2o, build an instance
    // of tracker_g2o.
    // To run with PTAM based code, build an instance
    // of Tracker.
    tracker_g2o tracker_;

    MapMaker::Parameters params_;

    const CameraParameters cameraParametersLeft_;
    const CameraParameters cameraParametersRight_;
    const double stereo_baseline_;

    RowMatcher rowMatcher_;

  // some system statistics

    size_t frames_since_last_kf_;
    size_t frames_number_;

  // helper functions

    // try to match features in the stereo frame to their corresponding
    // points on the map, and return those measurements.
    void matchToGlobal( const StereoFrame& frame,
                        std::vector<Measurement>& measurementsLeft,
                        std::vector<Measurement>& measurementsRight);

    void matchToPoints(const StereoFrame& frame, const std::vector<MapPoint*>& mapPoints, std::vector<Measurement>& measurementsLeft, std::vector<Measurement>& measurementsRight);

    void createNewPoints(StereoFrame& frame, const cv::Mat& imageLeft, std::vector<MapPoint*>& mapPoints);

    bool shouldBeKeyframe(const StereoFrame& frame);
};
