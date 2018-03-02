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
#include "CameraPose.hpp"
#include "Measurement.hpp"
#include "RectifiedCameraParameters.hpp"

#include <g2o/core/sparse_optimizer.h>

class G2ODriver
{
  public:

    typedef g2o::OptimizableGraph::Vertex Vertex;
    typedef g2o::OptimizableGraph::Edge Edge;

    G2ODriver();

    /**
     * Perform bundle adjustment for the previously loaded objects.
     *
     * @return
     *   return True if the process was explicitly interrupted by the user
     *   (using Break()). False otherwise.
     */
    bool Adjust(int maxIterations);

    // Driver SPTAM -> G2O

    /**
     * @brief TODO
     */
    class VertexData : public g2o::OptimizableGraph::Data
    {
      public:

        //! read the data from a stream
        // I'm not sure what this is for, I'll leave it blank (tfischer)
        virtual bool read(std::istream& is) override
        { return true; }

        //! write the data to a stream
        // I'm not sure what this is for, I'll leave it blank (tfischer)
        virtual bool write(std::ostream& os) const override
        { return true; }

        virtual void saveData(g2o::OptimizableGraph::Vertex& vertex) = 0;
    };

    Vertex* AddVertex(
      const CameraPose& cameraPose, const sptam::RectifiedCameraParameters& rectified_camera_parameters,
      const bool isFixed, VertexData* userData = nullptr
    );

    Vertex* AddVertex(const MapPoint& mapPoint, const bool marginalize ,const bool isFixed, VertexData* userData = nullptr);

    void AddEdge(int edgeId, Vertex* point, Vertex* keyFrame, const Measurement& meas);

    // Driver G2O -> SPTAM

    static Eigen::Vector3d GetPoint( g2o::HyperGraph::Vertex& vertex );

    static CameraPose GetPose( g2o::HyperGraph::Vertex& vertex, const Eigen::Matrix6d& covariance );

    Eigen::Matrix6d GetPoseCovariance( const g2o::OptimizableGraph::Vertex& pose_vertex );

    // ...

    inline const g2o::OptimizableGraph::EdgeContainer& activeEdges()
    { return optimizer_.activeEdges(); }

  private:

    // Id pool for the vertices
    int next_vertex_id_;

  // G2O optimizer

    g2o::SparseOptimizer optimizer_;

  // signal used by g2o to stop computing bundle adjustment

    bool optimizer_abort_request_;
    bool optimizer_abort_request_copy_;

    double gainTerminateThreshold_;

    double robust_cost_function_delta_;

  // helper functions

    Edge* BuildMonoEdge(const std::vector<double>& projection);
    Edge* BuildMonoEdgeRight(const std::vector<double>& projection);
    Edge* BuildStereoEdge(const std::vector<double>& projection);
};
