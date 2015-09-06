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

/**
 * @brief AddKeyFrameDistancePolicy
 * add a keyframe taking into account the distante w.r.t the last keyframe.
 * @param frame
 * @param map
 * @param keyFrameDistanceThreshold
 * @param framesBetweenKeyFrames
 * @param frames_since_last_kf
 * @return
 */
inline bool AddKeyFrameDistancePolicy(const StereoFrame& frame, const sptam::Map& map, const int keyFrameDistanceThreshold)
{

  const CameraPose& currentCameraPose = frame.GetCameraLeft().GetPose();

  const StereoFrame& closestKeyFrame = map.GetCurrentKeyFrame();

  cv::Point3d v3Diff = closestKeyFrame.GetPosition() - currentCameraPose.GetPosition();
  double distanceToClosestKeyFrame = sqrt( v3Diff.dot( v3Diff ) );

  // Heuristics to check if a key-frame should be added to the map
  return ( keyFrameDistanceThreshold < distanceToClosestKeyFrame );

}

inline bool compareX(std::pair<MapPoint*, Measurement> i, std::pair<MapPoint*, Measurement> j)
{

  Measurement& i_meas = i.second;
  Measurement& j_meas = j.second;

  const cv::Point2d& i_projection = i_meas.GetProjection();
  const cv::Point2d& j_projection = j_meas.GetProjection();

  return i_projection.x < j_projection.x;
}

inline int getMinX(std::map<MapPoint*, Measurement> mymap)
{
  std::pair<MapPoint*, Measurement> min = *min_element(mymap.begin(), mymap.end(),
                                                 &compareX);
  return min.second.GetProjection().x;
}

inline int getMaxX(std::map<MapPoint*, Measurement> mymap)
{
  std::pair<MapPoint*, Measurement> min = *max_element(mymap.begin(), mymap.end(),
                                                 &compareX);
  return min.second.GetProjection().x;
}

/**
 * @brief AddKeyFrameImageCoverPolicy
 * add a keyframe taking into account the percentage of image free of features matched
 * @param frame
 * @return
 */
inline bool AddKeyFrameImageCoverPolicy( const StereoFrame& frame,
                                         const double imageWidth,
                                         const double coverThreshold) {

  const std::map<MapPoint*, Measurement> &measurements = frame.GetMeasurementsLeft();

  double minX = getMinX(measurements);
  double maxX = getMaxX(measurements);

  double imageAreaMeasurementFree = (minX + (imageWidth - maxX)) / imageWidth;

  return imageAreaMeasurementFree > coverThreshold;
}

/**
 * @brief AddKeyFrameFeaturesPolicy
 * add a keyframe taking into account the percentege of keypoints matched over the total
 * @param frame
 * @return
 */
inline bool AddKeyFrameFeaturesPolicy( const StereoFrame& frame,
                                       const double featurePercentageThreshold) {


  double nMatches = frame.GetMeasurementsLeft().size(); // casting to double
  // declare variables
  std::vector<cv::KeyPoint> unMatchedKeyPoints;
  cv::Mat descriptors;
  std::vector<size_t> indexes;
  frame.GetFrameLeft().GetUnmatchedKeyPoints(unMatchedKeyPoints, descriptors, indexes);
  size_t unMatches = unMatchedKeyPoints.size();
  double totalKeypoints = nMatches + unMatches; // casting to double


  return (nMatches / totalKeypoints) < featurePercentageThreshold;
}


inline bool AddKeyFrameTrackingFeaturesPolicy( const StereoFrame& frame,
                                               const sptam::Map& map,
                                               const double featurePercentageThreshold ) {

  double nMatches = frame.GetMeasurementsLeft().size(); // casting to double
  const StereoFrame& closestKeyFrame = map.GetCurrentKeyFrame();
  double nReferenceKeyFrameMatches = closestKeyFrame.GetMeasurementsLeft().size();



  return (nMatches / nReferenceKeyFrameMatches) < featurePercentageThreshold;
}
