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

#include "MapPoint.hpp"

// TODO cambiarle el nombre a esto cuando desaparezca Measurement
struct MEAS
{
  // measured position in image frame
  cv::Point2d projection;

  // descriptor describing the feature
  cv::Mat descriptor;

  // performance values
  double distance;

  // index of measured point
  // TODO sacar esto de acá y devolver pair<idx, MEAS> donde sea necesario
  int index;
};

class Measurement
{
  public:

    /**
     * TODO esto existe solo porque bool StereoFrame::GetMeasurement(...)
     * toma un measurement como output parameter.
     * Ver si se puede cambiar eso.
     */
    Measurement(){}

    /**
     * The measurement descriptor is a reference to the MapPoint one.
     */
    Measurement(MapPoint* mapPoint, const cv::Point2d& projection);

    /**
     * The measurement descriptor is copied internally.
     */
    Measurement(MapPoint* mapPoint, const cv::Point2d& projection, const cv::Mat& descriptor);

    // MapPoint Id
    inline int GetMapPointId() const
    { return mapPoint->GetId(); }

    inline const cv::Point2d& GetProjection() const
    { return projection_; }

    // MapPoint descriptor
    inline const cv::Mat& GetDescriptor() const
    { return descriptor_; }

    // MapPoint asociado (este puntero es redundante dado que el KeyFrame tiene esta informacion).
    // El problema es que necesitaba acceso al punto durante al actualizacion de la camra en el tracking
    MapPoint* mapPoint;

  private:

    // Image feature position
    cv::Point2d projection_;

    // Image feature descriptor
    cv::Mat descriptor_;

  public:

    // Possible measurement sources
    typedef enum { SRC_TRIANGULATION, SRC_TRACKER, SRC_REFIND } Source_t;

    // Where did this measurement come from?
    Source_t source;
};

// What is the scale of a level?
inline int LevelScale(int nLevel)
{
  // is the same that use 2^nLevel
  return 1 << nLevel;
}

std::ostream& operator << (std::ostream& os, const Measurement& measurement);
