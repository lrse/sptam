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

#include "CameraPose.hpp"
#include "Measurement.hpp"

#include <g2o/core/sparse_optimizer.h>

class tracker_g2o
{
  public:
    tracker_g2o(
      const cv::Matx33d& intrinsicLeft,
      const cv::Matx33d& intrinsicRight,
      double stereo_baseline
    );

    /**
     * RefineCameraPose is the main working part of the tracker:
     * call this for every frame.
     * This will estimate the new pose of the system
     * and return a potential CameraPose.
     */
    CameraPose RefineCameraPose(const CameraPose& estimatedCameraPose,
      const std::map<MapPoint *, std::pair<Measurement, Measurement> > &measurementsStereo,
      const std::map<MapPoint *, Measurement> &measurementsLeft,
      const std::map<MapPoint *, Measurement> &measurementsRight
    );

protected:

    Eigen::Vector2d focal_length_;
    Eigen::Vector2d principal_point_;

    // G2O optimizer
    g2o::SparseOptimizer optimizer_;
    double gainTerminateThreshold_;

    // Calibration

    cv::Matx33d intrinsicLeft_;
    cv::Matx33d intrinsicRight_;

    double stereo_baseline_;

    g2o::OptimizableGraph::Vertex* BuildNewVertex(int vertex_id, const CameraPose& cameraPose, const bool isFixed);
    g2o::OptimizableGraph::Vertex* BuildNewVertex(int vertex_id, const MapPoint& mapPoint);
    g2o::OptimizableGraph::Edge* BuildNewMonoEdge(int pointId, int keyFrameId, const cv::Point2d& projection);
    g2o::OptimizableGraph::Edge* BuildNewMonoEdgeRight(int pointId, int keyFrameId, const cv::Point2d& projection);
    g2o::OptimizableGraph::Edge* BuildNewStereoEdge(int pointId, int keyFrameId, const cv::Point2d& projectionLeft, const cv::Point2d& projectionRight);

    CameraPose GetPose(int keyFrameId);


};
