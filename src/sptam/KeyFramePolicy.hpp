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

// ---------------------------------------------------------------------
// Helper functions

inline bool compareX(const Match& i, const Match& j)
{

  const Measurement& i_meas = i.measurement;
  const Measurement& j_meas = j.measurement;

  const std::vector<cv::KeyPoint>& i_keypoints = i_meas.GetKeypoints();
  const std::vector<cv::KeyPoint>& j_keypoints = j_meas.GetKeypoints();

  return i_keypoints[0].pt.x < j_keypoints[0].pt.x;
}

inline int getMinX(const std::list<Match>& measurements)
{
  const Match min = *min_element(measurements.begin(), measurements.end(), &compareX);
  return min.measurement.GetKeypoints()[0].pt.x;
}

inline int getMaxX(const std::list<Match>& measurements)
{
  const Match min = *max_element(measurements.begin(), measurements.end(), &compareX);
  return min.measurement.GetKeypoints()[0].pt.x;
}

// ---------------------------------------------------------------------
// Actual policies

/**
 * @brief TODO
 */
/*inline bool AddKeyFrameDistancePolicy(
  const StereoFrame& frame,
  const StereoFrame& closestKeyFrame,
  const int keyFrameDistanceThreshold
){
  Eigen::Vector3d v3Diff = closestKeyFrame.GetPosition() - frame.GetPosition();
  double distanceToClosestKeyFrame = sqrt( v3Diff.dot( v3Diff ) );

  // Heuristics to check if a key-frame should be added to the map
  return ( keyFrameDistanceThreshold < distanceToClosestKeyFrame );
}*/

/**
 * @brief TODO
 */
/*inline bool AddKeyFrameImageCoverPolicy(
  const StereoFrame& frame,
  const double imageWidth,
  const double coverThreshold
){
  const std::map<IPointData*, Measurement>& measurements = frame.GetMeasurements();

  double minX = getMinX(measurements);
  double maxX = getMaxX(measurements);

  double imageAreaMeasurementFree = (minX + (imageWidth - maxX)) / imageWidth;

  return imageAreaMeasurementFree > coverThreshold;
}*/

/**
 * @brief TODO
 */
/*inline bool AddKeyFrameFeaturesPolicy(
  const StereoFrame& frame,
  const double featurePercentageThreshold
){
  double nMatches = frame.GetMeasurements().size(); // casting to double
  // declare variables
  std::vector<cv::KeyPoint> unMatchedKeyPoints;
  cv::Mat descriptors;
  std::vector<size_t> indexes;
  frame.GetFrameLeft().GetUnmatchedKeyPoints(unMatchedKeyPoints, descriptors, indexes);
  size_t unMatches = unMatchedKeyPoints.size();
  double totalKeypoints = nMatches + unMatches; // casting to double

  return (nMatches / totalKeypoints) < featurePercentageThreshold;
}*/

/**
 * @brief TODO
 */
inline bool AddKeyFrameTrackingFeaturesPolicy(
  const StereoFrame& frame,
  const std::list<Match>& measurements,
  const sptam::Map::KeyFrame& closestKeyFrame,
  const double featurePercentageThreshold
){
  double nMatches = measurements.size(); // casting to double
  double nReferenceKeyFrameMatches = closestKeyFrame.measurements().size(); // casting to double

  //std::cout << "frame tracks " << nMatches << " and reference frame tracks " << nReferenceKeyFrameMatches << std::endl;

  return (nMatches / nReferenceKeyFrameMatches) < featurePercentageThreshold;
}
