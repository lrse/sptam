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

#include "Frame.hpp"
#include "utils/macros.hpp"
#include "utils/projective_math.hpp"

inline std::vector<cv::Point2d> project(const cv::Matx34d& projection, const std::vector<cv::Point3d>& points)
{
  std::vector<cv::Point2d> ret;
  ret.reserve( points.size() );

  for ( auto point : points )
    ret.push_back( project( projection, point ) );

  return ret;
}

Frame::Frame(const Camera& camera, const ImageFeatures& imageFeatures)
  : camera_( camera ), imageFeatures_( imageFeatures )
{}

void Frame::AddMeasurement(const Measurement& measurement)
{
  measurements_.insert(std::pair<MapPoint*, Measurement>(
    measurement.mapPoint, measurement
  ));

  measurement.mapPoint->IncreaseMeasurementCount();
}

void Frame::AddMeasurement(int meas_idx, const MapPoint& mapPoint, Measurement::Source_t source)
{
  const cv::KeyPoint& keypoint = imageFeatures_.GetKeypoint( meas_idx );

  cv::Point2d projection = keypoint.pt;

  // A match is added to the result
  Measurement newMatch(
    // TODO feo castear!
    const_cast<MapPoint*>( &mapPoint ),
    projection,
    imageFeatures_.GetDescriptor( meas_idx )
  );

  newMatch.source = source;

  imageFeatures_.SetMatchedKeyPoint( meas_idx );

  AddMeasurement( newMatch );
}

size_t Frame::RemoveMeasurement(MapPoint* mapPoint)
{
  mapPoint->DecreaseMeasurementCount();

  return measurements_.erase( mapPoint );
}

size_t Frame::RemoveBadPointMeasurements()
{
  size_t measRemovedNumber = 0;
  for( auto it = measurements_.begin(); it != measurements_.end(); ) {
    MapPoint* mapPoint = it -> first;
    if ( mapPoint->IsBad() ) {
      it = measurements_.erase(it);
      measRemovedNumber++;
    }
    else ++it;
  }

  return measRemovedNumber;
}

bool Frame::GetMeasurement(const MapPoint* mapPoint, Measurement& measurement) const
{
  // TODO: feo el cast, aunque solo se usa para indexar
  auto it = measurements_.find( const_cast<MapPoint*>( mapPoint ) );

  if ( it == measurements_.end() )
    return false;

  measurement = it->second;
  return true;
}

void Frame::FindMatches(
  const std::vector<cv::Point3d>& points,
  const std::vector<cv::Mat>& descriptors,
  const cv::DescriptorMatcher& descriptorMatcher,
  const double matchingDistanceThreshold, const size_t matchingNeighborhoodThreshold,
  std::vector<MEAS>& measurements
) const
{
  // reserve space
  measurements.reserve( points.size() );

  std::vector<cv::Point2d> featurePredictions = project( camera_.GetProjection(), points );

  forn ( i, featurePredictions.size() ) {

    cv::Point2d& point = featurePredictions[ i ];
    cv::Mat descriptor = descriptors[ i ];

    int index = imageFeatures_.FindMatch(point, descriptor, descriptorMatcher, matchingDistanceThreshold, matchingNeighborhoodThreshold);

    if ( 0 <= index ) {

      imageFeatures_.SetMatchedKeyPoint( index );

      const cv::KeyPoint& matchedKeypoint = imageFeatures_.GetKeypoint( index );

      // A match is added to the result
      MEAS meas;
      meas.projection = matchedKeypoint.pt;
      meas.descriptor = imageFeatures_.GetDescriptor( index );

      meas.index = i;

      measurements.push_back( meas );
    }
  }
}

int Frame::FindMatch(
  const MapPoint& mapPoint,
  const cv::DescriptorMatcher& descriptorMatcher,
  const double matchingDistanceThreshold,
  const size_t matchingNeighborhoodThreshold
) const
{
  // Project Map Points to the keyFrame
  cv::Point2d prediction = project(camera_.GetProjection(), mapPoint.GetPosition());

  return imageFeatures_.FindMatch(prediction, mapPoint.GetDescriptor(), descriptorMatcher, matchingDistanceThreshold, matchingNeighborhoodThreshold);
}
