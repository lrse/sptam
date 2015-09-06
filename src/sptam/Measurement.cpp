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

#include "Measurement.hpp"

Measurement::Measurement(MapPoint* _mapPoint, const cv::Point2d& projection)
  : mapPoint( _mapPoint ), projection_( projection )
  , descriptor_( mapPoint->GetDescriptor() )
{}

Measurement::Measurement(MapPoint* mapPoint, const cv::Point2d& projection, const cv::Mat& descriptor)
  : mapPoint( mapPoint ), projection_( projection )
{
  descriptor.copyTo( descriptor_ );
}

std::ostream& operator << (std::ostream& os, const Measurement& measurement)
{
  return os << "id: "  << measurement.GetMapPointId()
    << " point: "<< measurement.mapPoint->GetPosition()
    << " proj: "  << measurement.GetProjection() << std::endl;
}
