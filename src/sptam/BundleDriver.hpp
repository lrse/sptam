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

#include "StereoFrame.hpp"

#include <g2o/core/sparse_optimizer.h>

/**
 * The driver is an interface between the SPTAM framework and an external
 * bundle adjustment optimization routine.
 * optimizer setup and data format conversions are handled here.
 */
class BundleDriver
{
  public:

    BundleDriver(
      const cv::Matx33d& intrinsicLeft,
      const cv::Matx33d& intrinsicRight,
      const double stereo_baseline);

    /**
     * Add the data to be used in the next adjustment call ( Adjust )
     */
    void SetData(const std::vector<StereoFrame*>& views, const std::vector<MapPoint*>& points);

    /**
     * Add the data to be used in the next adjustment call ( Adjust )
     */
    void SetData(const std::set<StereoFrame*>& adjustViews, const std::set<StereoFrame*>& fixedViews, const std::set<MapPoint*>& points);

    /**
     * Perform bundle adjustment for the given parameters.
     * return true if BA finish, false if it was interrupted
     */
    bool Adjust(int maxIterations);

    /**
     * Load the parameters adjusted on the last adjustment call ( Adjust )
     */
    void SavePoints();
    void SaveCameras();

    /**
     * Handle Bad Measurements
     */
    std::list< std::pair<StereoFrame*, MapPoint*> > GetBadMeasurements();

    /**
     * Interrupt the bundle adjustment if there is one in progress.
     * This function is (probably) called from another thread in which
     * the Adjust(...) function is running.
     */
    void Break();

    private:

    // G2O optimizer
    g2o::SparseOptimizer optimizer_;

    // signal used by g2o to stop computing bundle adjustment
    bool bundleAbortRequested_;
    bool bundleAbortRequestedCopy_;

  // projection variables

    Eigen::Vector2d focal_length_;
    Eigen::Vector2d principal_point_;

    double baseline_;
    double gainTerminateThreshold_;

    void Clear();

  // helpers

    // The bundle adjuster does different accounting of keyframes and map points;
    // Translation maps are stored:
    struct IdMaps
    {
      std::map<MapPoint*, int> point_to_BundleId;
      std::map<int, MapPoint*> BundleId_to_point;
      std::map<StereoFrame*, int> view_to_BundleId;
      std::map<int, StereoFrame*> BundleId_to_view;
    };

    IdMaps id_maps_;

    /**
     * Add Measurement to the Bundle using the id_maps_
     */
    void AddMeasToBundle();

  // Driver SPTAM -> G2O

    g2o::OptimizableGraph::Vertex* BuildNewVertex(int vertex_id, const CameraPose& cameraPose, const bool isFixed);

    g2o::OptimizableGraph::Vertex* BuildNewVertex(int vertex_id, const MapPoint& mapPoint, const bool marginalize ,const bool isFixed);

    g2o::OptimizableGraph::Edge* BuildNewMonoEdge(
      int pointId, int keyFrameId, const cv::Point2d& projection
    );

    g2o::OptimizableGraph::Edge* BuildNewMonoEdgeRight(
      int pointId, int keyFrameId, const cv::Point2d& projection
    );

    g2o::OptimizableGraph::Edge* BuildNewStereoEdge(
      int pointId, int keyFrameId,
      const cv::Point2d& projectionLeft,
      const cv::Point2d& projectionRight
    );

  // Driver G2O -> SPTAM

    cv::Vec3d GetPoint( int pointId );

    CameraPose GetPose( int keyFrameId );
};
