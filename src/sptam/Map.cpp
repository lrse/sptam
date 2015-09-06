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

#include "Map.hpp"
#include "utils/Profiler.hpp"

#include <list>

namespace sptam
{

Map::Map()
  : lastMapPointId_( 0 ), lastKeyFrameId_( 0 )
{}

void Map::AddKeyFrame(StereoFrame* keyFrame)
{
//  keyFrame->SetId( lastKeyFrameId_ );
  lastKeyFrameId_++;
  keyFrames_.push_back( keyFrame );
}

void Map::AddMapPoint(MapPoint* mapPoint)
{
  mapPoint->SetId( lastMapPointId_ );
  lastMapPointId_++;
  mapPoints_.push_back( mapPoint );
  return;
}

void Map::AddMapPoints(const std::vector<MapPoint*>& new_points)
{
  mapPoints_.reserve( mapPoints_.size() + new_points.size() );
  mapPoints_.insert( mapPoints_.end(), new_points.begin(), new_points.end() );
}

void Map::RemoveBadPoints()
{
  #ifdef SHOW_PROFILING
    double start_lock_handle = GetSeg();
  #endif

  points_mutex_.lock();

  #ifdef SHOW_PROFILING
    double end_lock_handle = GetSeg();
    WriteToLog(" ba LockPointsMapper: ", start_lock_handle, end_lock_handle);
  #endif

  // http://en.wikipedia.org/wiki/Erase-remove_idiom
  // TODO (thomas) aca estamos perdiendo toda referencia a los punteros
  // hay que hacer un delete, aunque lo ideal sería no guardar punteros pelados

  auto it_bad = std::remove_if( std::begin(mapPoints_), std::end(mapPoints_), std::mem_fn(&MapPoint::IsBad) );

  std::list<MapPoint*> bad_points(it_bad, std::end(mapPoints_));

  mapPoints_.erase(it_bad, std::end(mapPoints_));

  points_mutex_.unlock();

  // remove deleted point measurements
  // All points marked as bad will be erased - so erase all records
  // from keyframes in which they might have been measured.

  for( auto& keyFrame : keyFrames_ )
    for (auto it=it_bad; it!=std::end(mapPoints_); ++it)
      keyFrame->RemoveMeasurement( *it );
}

void Map::RemoveBadKeyFrames()
{
  // http://en.wikipedia.org/wiki/Erase-remove_idiom
  keyFrames_.erase( std::remove_if( std::begin(keyFrames_), std::end(keyFrames_), std::bind(&Map::IsBad, this, std::placeholders::_1) ), std::end(keyFrames_) );
}

} // namespace sptam
