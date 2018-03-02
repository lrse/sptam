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

#include "MapPoint.hpp"
#include "StereoFrame.hpp"
#include "Measurement.hpp"
#include "utils/CovisibilityGraph.hpp"
#include "utils/Iterable.hpp"

#include <boost/thread/shared_mutex.hpp>

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

    typedef CovisibilityGraph<StereoFrame, MapPoint, Measurement> Graph;

    typedef Graph::KeyFrame KeyFrame;
    typedef Graph::MapPoint Point;
    typedef Graph::Measurement Meas;

    typedef Graph::SharedKeyFrame SharedKeyFrame;
    typedef Graph::SharedMapPoint SharedPoint;
    typedef Graph::SharedMeasurement SharedMeas;

    typedef Graph::SharedMapPointSet SharedMapPointSet;
    typedef Graph::SharedKeyFrameSet SharedKeyFrameSet;
    typedef Graph::SharedMeasurementSet SharedMeasurementSet;

    typedef Graph::SharedMapPointList SharedMapPointList;
    typedef Graph::SharedKeyFrameList SharedKeyFrameList;

//    typedef Graph::MapPointRefList PointRefs;

    SharedKeyFrame AddKeyFrame(const StereoFrame& frame)
    { return graph_.addKeyFrame( frame ); }

    void RemoveKeyFrame(const SharedKeyFrame& keyFrame)
    { graph_.removeKeyFrame( keyFrame ); }

    SharedPoint AddMapPoint(const MapPoint& mapPoint)
    { return graph_.addMapPoint( mapPoint ); }

    void RemoveMapPoint(const SharedPoint& mapPoint)
    { graph_.removeMapPoint( mapPoint ); }

    void addMeasurement(const SharedKeyFrame& keyFrame, const SharedPoint& mapPoint, const Measurement& measurement)
    { graph_.addMeasurement( keyFrame, mapPoint, measurement ); }

    void removeMeasurement(const SharedMeas& measurement)
    { graph_.removeMeasurement( measurement ); }

    inline void getLocalMap(const sptam::Map::SharedMapPointSet& trackedPoints, SharedMapPointSet& localMap, SharedKeyFrameSet& localKeyFrames, SharedKeyFrame& referenceKeyFrame )
    { graph_.getLocalMap( trackedPoints, localMap, localKeyFrames, referenceKeyFrame ); }

    const SharedKeyFrameList& getKeyframes() const
    { return graph_.getKeyframes(); }

    const SharedMapPointList& getMapPoints() const
    { return graph_.getMapPoints(); }

    //void RemoveBadPoints();

    /* Map points use an internal locking system, so they're thread-safe.
     * Yet the map isn't, so we use an external mutex for
     * general map manipulation.
     * Tracking, Mapping and LoopClosing have to ask for the lock when:
     * Read-Lock: Get keyframes/mappoints, localMap or asking for any internal parameter.
     * Write-Lock: Adding points/frames, Remove points/frames */
    mutable boost::shared_mutex map_mutex_; // Map and Keyframe mutex

  private:

    Graph graph_;
};

} // namespace sptam
