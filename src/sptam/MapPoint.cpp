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

#include "MapPoint.hpp"
#include "utils/projective_math.hpp"

MapPoint::MapPoint(const Eigen::Vector3d& position, const Eigen::Vector3d& normal, const cv::Mat& descriptor, const Eigen::Matrix3d& covariance)
  : position_( position )
  , covariance_( covariance )
  , normal_( normal )
  , outlierCount_( 0 )
  , inlierCount_( 0 )
  , projectionCount_( 0 )
  , measurementCount_( 0 ) // is zero because, measurementCount_ is incremented always by Frame::AddMeasuement()
  , color_(255,255,255)
{
  // TODO: do we have to clone? can't we just assign it?
  descriptor.copyTo(descriptor_);
}

/* When a points is added to the Map, has to be copyed, but the shared_mutex does cannot be
 * copied. Thats why we have to specify that the new points will have a separate mutex */
MapPoint::MapPoint(const MapPoint& mapPoint)
: mpoint_mutex_() // The copy has a diferent mutex!
, position_( mapPoint.position_ )
, covariance_( mapPoint.covariance_ )
, normal_( mapPoint.normal_ )
, outlierCount_( mapPoint.outlierCount_ )
, inlierCount_( mapPoint.inlierCount_ )
, projectionCount_( mapPoint.projectionCount_ )
, measurementCount_( mapPoint.measurementCount_ )
, color_( mapPoint.color_ )
{
  mapPoint.GetDescriptor().copyTo(descriptor_);
}
