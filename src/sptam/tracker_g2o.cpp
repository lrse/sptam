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

#include "tracker_g2o.hpp"
#include "types_sba_extension.hpp"

#include "utils/macros.hpp"
#include "utils/pose_covariance.hpp"
#include "utils/projective_math.hpp"
#include "utils/projection_derivatives.hpp"

#include <Eigen/Eigenvalues>

// ===================== //
// Some magic numbers :) //
// ===================== //

// Minimum number of measurements required for tracking to operate correctly.
#define MIN_MEAS_FOR_TRACKING 10

// Maximum number of iterations we allow the BA to run.
#define BA_MAX_ITERATIONS 10

CameraPose tracker_g2o::RefineCameraPose(
  const CameraPose& estimatedCameraPose,
  const sptam::RectifiedCameraParameters& rectified_camera_parameters,
  const std::list<Match>& measurements
)
{
  size_t measurements_num = measurements.size();

  // We need at least 4 measurements to get a solution.
  // Make sure there are a bunch more just to be robust.
  if( measurements_num < MIN_MEAS_FOR_TRACKING )
    throw not_enough_points();

  // freeing the graph memory
  g2o_driver_.Clear();

  G2ODriver::Vertex* pose_vertex = g2o_driver_.AddVertex(estimatedCameraPose, rectified_camera_parameters, false);

  // Add the points' 3D position

  for( const auto& match : measurements )
  {
    G2ODriver::Vertex* point_vertex = g2o_driver_.AddVertex(*match.mapPoint, false, true);

    // trivial edge id, since we won't need it for anything
    g2o_driver_.AddEdge(0, point_vertex, pose_vertex, match.measurement);
  }

  if (!g2o_driver_.Adjust( BA_MAX_ITERATIONS ))
    std::cout << "WARNING: reached BA_MAX_ITERATIONS in tracker during refine" << std::endl;

  return G2ODriver::GetPose( *pose_vertex, GetPoseCovariance(*pose_vertex) );
}

Eigen::Matrix6d tracker_g2o::GetPoseCovariance(g2o::HyperGraph::Vertex& pose_vertex)
{
  const g2o::OptimizableGraph::Vertex& pose_graph_vertex = dynamic_cast<const g2o::OptimizableGraph::Vertex&>( pose_vertex );

  return g2o_driver_.GetPoseCovariance( pose_graph_vertex );
}
