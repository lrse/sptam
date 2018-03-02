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

#include "../../Match.hpp"

#include "opencv2/core/version.hpp"
#if CV_MAJOR_VERSION == 2
  #include <opencv2/highgui/highgui.hpp>
#elif CV_MAJOR_VERSION == 3
  #include <opencv2/highgui.hpp>
#endif

static const cv::Scalar COLOR_BLACK(0,0,0);
static const cv::Scalar COLOR_RED(0,0,255);
static const cv::Scalar COLOR_GREEN(0,255,0);
static const cv::Scalar COLOR_BLUE(255,0,0);
static const cv::Scalar COLOR_CYAN(255,255,0);
static const cv::Scalar COLOR_MAGENTA(255,0,255);
static const cv::Scalar COLOR_YELLOW(0,255,255);
static const cv::Scalar COLOR_WHITE(255,255,255);
static const cv::Scalar COLOR_GREY(127,127,127);


/**
 * Draw a frame with its measurements and their corresponding
 * expected projections.
 */
cv::Mat drawFrame(const sptam::Map::KeyFrame& frame, const cv::Mat& image);

cv::Mat drawFrame(const sptam::Map::KeyFrame& frame);

/**
 * @brief ...
 *   T is required to implement the IPointData interface.
 */
void drawMeasuredFeatures(const StereoFrame& frame, const std::list<Match>& measurements, cv::Mat& imgL, cv::Mat& imgR);

void drawMeasuredFeatures(const StereoFrame& frame, const std::list<Match>& measurements, cv::Mat& img, bool left);

void drawTrackedFrames(const StereoFrame& frame, cv::Mat& outImageLeft, cv::Mat& outImageRight);

cv::Mat makeStereoWindow(const cv::Mat& imgL, const cv::Mat& imgR, cv::Mat& outImageLeft, cv::Mat& outImageRight, bool horizontal = true);

void makeColorCopy(const cv::Mat& in, cv::Mat& out);

template<class T>
void drawProjectionCovariances(const Frame& frame, cv::Mat& image, const T& points);

void drawGrid(cv::Mat& image, const size_t cell_size, const cv::Scalar& color = COLOR_GREY);

void drawProjections(cv::Mat& image, const Eigen::Matrix34d& projection, const ConstIterable<sptam::Map::Point>& mapPoints, const cv::Scalar& color = COLOR_BLUE);

/**
 * Save an image file for a frame with its measurements and their corresponding
 * expected projections.
 */
void saveFrame(const Frame& frame, const std::string& fileName);

/**
 * save Stereo KeyFrame with its matches
 */

//void saveStereoKeyFrame(const sptam::Map::KeyFrame& keyFrame, const std::string fileName);

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
