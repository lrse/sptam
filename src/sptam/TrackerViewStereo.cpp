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

#include "TrackerViewStereo.hpp"
#include "utils/draw/Draw.hpp"

#define DRAW_COVARIANCES 0

void sptam::TrackerViewStereo::draw(
  const StereoFrame& frame, const sptam::Map::SharedMapPointList& filtered_points,
  const std::list<Match>& measurements, const Parameters& params, bool before_refine)
{
#ifdef SHOW_TRACKED_FRAMES
  /* check if no output is expected */
  if (draw_output == DRAW_NONE)
    return;

  /* check if currently requested output is expected */
  if ((before_refine &&
       !(draw_output & DRAW_BEFORE_REFINE_LEFT) &&
       !(draw_output & DRAW_BEFORE_REFINE_RIGHT))
      ||
      (!before_refine &&
       !(draw_output & DRAW_AFTER_REFINE_LEFT) &&
       !(draw_output & DRAW_AFTER_REFINE_RIGHT)
      ))
    return;

  /* check if input images are valid */
  if (image_left_.empty() || image_right_.empty())
    return;    

  cv::Mat left_out, right_out;

  /* create stereo outputs if required */
  if (before_refine && (draw_output & DRAW_BEFORE_REFINE_STEREO))
    stereoFrameBeforeRefine = makeStereoWindow(image_left_, image_right_, left_out, right_out);
  else if (!before_refine && (draw_output & DRAW_AFTER_REFINE_STEREO))
    stereoFrameAfterRefine = makeStereoWindow(image_left_, image_right_, left_out, right_out);

  /* create individual outputs if required */
  if ((draw_output & DRAW_BEFORE_REFINE_LEFT) || (draw_output & DRAW_AFTER_REFINE_LEFT))
    makeColorCopy(image_left_, left_out);
  if ((draw_output & DRAW_BEFORE_REFINE_RIGHT) || (draw_output & DRAW_AFTER_REFINE_RIGHT))
    makeColorCopy(image_right_, right_out);

  /* map outputs */
  if (before_refine)
  {
    leftFrameBeforeRefine = left_out;
    rightFrameBeforeRefine = right_out;
  }
  else
  {
    leftFrameAfterRefine = left_out;
    rightFrameAfterRefine = right_out;
  }

  if (before_refine)
  {
    if (draw_output & DRAW_BEFORE_REFINE_LEFT) drawGrid(left_out, params.matchingCellSize);
    if (draw_output & DRAW_BEFORE_REFINE_RIGHT) drawGrid(right_out, params.matchingCellSize);
  }

  if ((draw_output & DRAW_BEFORE_REFINE_LEFT) || (draw_output & DRAW_AFTER_REFINE_LEFT))
    drawProjections(left_out, frame.GetFrameLeft().GetProjection(), ConstSharedPtrListIterable<sptam::Map::Point>::from( filtered_points ));
  if ((draw_output & DRAW_BEFORE_REFINE_RIGHT) || (draw_output & DRAW_AFTER_REFINE_RIGHT))
    drawProjections(right_out, frame.GetFrameRight().GetProjection(), ConstSharedPtrListIterable<sptam::Map::Point>::from( filtered_points ));

#if DRAW_COVARIANCES
  if (before_refine)
  {
    if (draw_output & DRAW_BEFORE_REFINE_LEFT) drawProjectionCovariances(frame.GetFrameLeft(), left_out, filtered_points);
    if (draw_output & DRAW_BEFORE_REFINE_RIGHT) drawProjectionCovariances(frame.GetFrameRight(), right_out, filtered_points);
  }
#endif

  // extract and draw unmatched keypoints
  if (before_refine)
  {
    std::vector<cv::KeyPoint> keyPointsLeft, keyPointsRight;
    cv::Mat descriptorsLeft, descriptorsRight;
    std::vector<size_t> indexesLeft, indexesRight;

    if (draw_output & DRAW_BEFORE_REFINE_LEFT)
    {
      frame.GetFrameLeft().GetUnmatchedKeyPoints(keyPointsLeft, descriptorsLeft, indexesLeft);
      cv::drawKeypoints(left_out, keyPointsLeft, left_out, COLOR_CYAN);
    }

    if (draw_output & DRAW_BEFORE_REFINE_RIGHT)
    {
      frame.GetFrameRight().GetUnmatchedKeyPoints(keyPointsRight, descriptorsRight, indexesRight);
      cv::drawKeypoints(right_out, keyPointsRight, right_out, COLOR_CYAN);
    }
  }

  if ((draw_output & DRAW_BEFORE_REFINE_STEREO) || (draw_output & DRAW_AFTER_REFINE_STEREO))
    drawMeasuredFeatures(frame, measurements, left_out, right_out);
  else if ((draw_output & DRAW_BEFORE_REFINE_LEFT) || (draw_output & DRAW_AFTER_REFINE_LEFT))
    drawMeasuredFeatures(frame, measurements, left_out, true);
  else if ((draw_output & DRAW_BEFORE_REFINE_RIGHT) || (draw_output & DRAW_AFTER_REFINE_RIGHT))
    drawMeasuredFeatures(frame, measurements, right_out, false);
#endif
}

cv::Vec3b sptam::TrackerViewStereo::featureColor(const Measurement& meas) const
{
  /* obtain color for this point according to its corresponding keypoint in the corresponding image */
  cv::Mat img = (meas.GetType() == Measurement::RIGHT) ? image_right_ : image_left_;

  cv::Point2f p = meas.GetMainKeypoint().pt;

  return (img.channels() == 1)
    ? cv::Vec3b(1,1,1) * img.at<unsigned char>((int)p.y, (int)p.x)
    : img.at<cv::Vec3b>((int)p.y, (int)p.x);
}

void sptam::TrackerViewStereo::enableDrawOutput(TrackerViewStereo::DrawOutput output)
{
  draw_output = (DrawOutput)(draw_output | output);
}
