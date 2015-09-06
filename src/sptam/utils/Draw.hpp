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

#include "../StereoFrame.hpp"
#include <opencv2/highgui/highgui.hpp>

static const cv::Scalar COLOR_BLACK(0,0,0);
static const cv::Scalar COLOR_RED(0,0,255);
static const cv::Scalar COLOR_GREEN(0,255,0);
static const cv::Scalar COLOR_BLUE(255,0,0);
static const cv::Scalar COLOR_CYAN(255,255,0);
static const cv::Scalar COLOR_MAGENTA(255,0,255);
static const cv::Scalar COLOR_YELLOW(0,255,255);
static const cv::Scalar COLOR_WHITE(255,255,255);

/**
 * Draw a frame with its measurements and their corresponding
 * expected projections.
 */
cv::Mat drawFrame(const Frame& frame, const cv::Mat& image);

cv::Mat drawFrame(const Frame& frame);

cv::Mat drawTrackedFrames(const StereoFrame& frame, const cv::Mat& imgL, const cv::Mat& imgR, cv::Mat& outImageLeft, cv::Mat& outImageRight);

cv::Mat drawTrackedFrames(const StereoFrame& frame, const cv::Mat& imgL, const cv::Mat& imgR);

void drawFeatures(const cv::Matx34d& projection, const std::vector<cv::Point3d>& points, const std::vector<MEAS>& features,
                  const cv::Scalar& color_proj, const cv::Scalar& color_feat, cv::Mat& image);

void drawFeatures(const cv::Matx34d& projection, const std::vector<Measurement>& measurements,
                  const cv::Scalar& color_proj, const cv::Scalar& color_feat, cv::Mat& image);

/**
 * Save an image file for a frame with its measurements and their corresponding
 * expected projections.
 */
void saveFrame(const Frame& frame, const std::string& fileName);

/**
 * save Stereo KeyFrame with its matches
 */

void saveStereoKeyFrame(const StereoFrame& keyFrame, const std::string fileName);

/**
 * DrawEpipolarLines dibuja todas las lineas epipolares en una imagen 
 * dado los puntos y la matriz fundamental
 */
void DrawEpipolarLine(
  cv::Mat& image,
  const std::vector<cv::Point2f>& inlierPointsLeft,
  const std::vector<cv::Point2f>& inlierPointsRight,
  const cv::Matx33d& fundamentalMatrix,
  std::vector<cv::Vec3f>& epipolarLinesRight
);

/***/
void DrawOneEpipolarLine(cv::Mat& image, cv::Vec3f& line);

void DrawMatches(const cv::Mat& img1, const std::vector<cv::KeyPoint>& keypoints1,
                 const cv::Mat& img2, const std::vector<cv::KeyPoint>& keypoints2,
                 const std::vector<cv::DMatch>& matches, cv::Mat& outImage);
