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

#include "utils/draw/TrackerView.hpp"

#include "opencv2/core/version.hpp"
#if CV_MAJOR_VERSION == 2
 #include <opencv2/features2d/features2d.hpp>
#elif CV_MAJOR_VERSION == 3
#include <opencv2/features2d.hpp>
#endif

namespace sptam
{

class TrackerViewStereo : public TrackerView
{
  public:

    TrackerViewStereo(const cv::Mat& image_left, const cv::Mat& image_right)
      : draw_output( DRAW_NONE ), image_left_( image_left ), image_right_( image_right )
    {}

    void draw(const StereoFrame& frame, const sptam::Map::SharedMapPointList& filtered_points,
      const std::list<Match>& measurements, const Parameters& params, bool before_refine) override;

    cv::Vec3b featureColor(const Measurement& meas) const override;

    cv::Mat stereoFrameBeforeRefine, stereoFrameAfterRefine,
            leftFrameBeforeRefine, rightFrameBeforeRefine,
            leftFrameAfterRefine, rightFrameAfterRefine;

    /**
     * Specifies what kind of output is expected of the tracker
     */
    enum DrawOutput
    {
      DRAW_NONE                  = 0,
      DRAW_BEFORE_REFINE_LEFT    = (1 << 0),
      DRAW_BEFORE_REFINE_RIGHT   = (1 << 1),
      DRAW_BEFORE_REFINE_STEREO  = (DRAW_BEFORE_REFINE_LEFT | DRAW_BEFORE_REFINE_RIGHT),
      DRAW_AFTER_REFINE_LEFT     = (1 << 2),
      DRAW_AFTER_REFINE_RIGHT    = (1 << 3),
      DRAW_AFTER_REFINE_STEREO   = (DRAW_AFTER_REFINE_LEFT | DRAW_AFTER_REFINE_RIGHT),
    };

    void enableDrawOutput(DrawOutput output);

  private:

    DrawOutput draw_output;

    const cv::Mat image_left_, image_right_;
};

}
