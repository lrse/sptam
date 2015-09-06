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

#include <mutex>

#include "MapPoint.hpp"
#include "StereoFrame.hpp"

// TODO esto debería ser un parámetro del programa creo.
// Lo pongo acá porque me da fiaca, perdón :( (thomas)
#define MIN_NUM_MEAS 10

namespace sptam
{

/**
 * This class represents the visibility map of the environment.
 * It is composed by 3D points in space, 6DOF camera views, and
 * the relationships between them, called measurements.
 */
class Map
{
  public:

    Map();

    // Add a KeyFrame to the Map
    void AddKeyFrame(StereoFrame* keyFrame);

    // Add a MapPoint to the Map
    void AddMapPoint(MapPoint* mapPoint);

    // Add a bunch of new points to the map
    void AddMapPoints(const std::vector<MapPoint*>& new_points);

    // 
    inline StereoFrame* GetCurrentKeyFrame()
    { return keyFrames_.back(); }

		inline const StereoFrame& GetCurrentKeyFrame() const
    { return *keyFrames_.back(); }

    // return the total number of mapPoints
    inline size_t nMapPoints() const
    { return mapPoints_.size(); }

    // return the total number of KeyFrames
    inline size_t nKeyFrames() const
    { return keyFrames_.size(); }

    inline const std::vector<MapPoint*>& GetMapPoints() const
    { return mapPoints_; }

    inline const std::vector<StereoFrame*>& GetKeyFrames() const
    { return keyFrames_; }

    void RemoveBadPoints();

    void RemoveBadKeyFrames();

    // S-PTAM will try to access the Point structure from the Tracker,
    // to localize incoming frames, and in parallell may try to update
    // the refined structure from the Mapper.
    // This mutex should be used to allow acces in such cases.
    mutable std::mutex points_mutex_;

  private:

    std::vector<StereoFrame*> keyFrames_;

    std::vector<MapPoint*> mapPoints_;

    // Id pools for keyframes and mapPoints
    // TODO poner algo mas copado
    int lastMapPointId_;
    int lastKeyFrameId_;

    inline bool IsBad(StereoFrame* keyFrame)
    { return keyFrame->GetNumberOfMeasurements() < MIN_NUM_MEAS; }
};

} // namespace sptam
