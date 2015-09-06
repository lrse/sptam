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

#include "MapPoint.hpp"
#include "utils/projective_math.hpp"

MapPoint::MapPoint(const cv::Point3d& position, const cv::Point3d& normal, const cv::Mat& descriptor)
  : id_( 0 )
  , position_( position )
  , normal_( normal )
  , outlierCount_( 0 )
  , inlierCount_( 2 ) // the triangulation of point imply that it was inlier in two views
  , projectionCount_( 2 ) // the triangulation of point imply that it has been projected by two views
  , measurementCount_( 0 ) // is zero because, measurementCount_ is incremented always by Frame::AddMeasuement()
  , color(0,0,0) // <----- for visualization
{
  descriptor.copyTo(descriptor_);
}

std::ostream& operator << (std::ostream& os, const MapPoint& mapPoint)
{
  return os << "id:" << mapPoint.GetId() << " pt: " << mapPoint.GetPosition();
}
