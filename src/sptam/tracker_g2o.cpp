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

#include "tracker_g2o.hpp"
#include "utils/projective_math.hpp"
#include "utils/cv2eigen.hpp"
#include "types_sba_extension.hpp"

#include <opencv2/core/eigen.hpp>

// G2O Headers
#include <g2o/config.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/types/icp/types_icp.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/solvers/pcg/linear_solver_pcg.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include <g2o/core/sparse_optimizer_terminate_action.h>

// constructor of class Tracker_g2o
tracker_g2o::tracker_g2o(
  const cv::Matx33d& intrinsicLeft,
  const cv::Matx33d& intrinsicRight,
  double stereo_baseline
)
  : intrinsicLeft_( intrinsicLeft )
  , intrinsicRight_( intrinsicLeft )
  , stereo_baseline_( stereo_baseline )
{
  focal_length_ = cv2eigen( getFocalLength( intrinsicLeft ) );
  principal_point_ = cv2eigen( getPrincipalPoint( intrinsicLeft ) );

  /// SETUP G2O ///

  optimizer_.setVerbose( false );

  g2o::BlockSolver_6_3::LinearSolverType* linearSolver
    = new g2o::LinearSolverPCG<g2o::BlockSolver_6_3::PoseMatrixType>();

  g2o::BlockSolver_6_3* solver_ptr = new g2o::BlockSolver_6_3( linearSolver );

  // Choosing the method to use by the optimizer
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( solver_ptr );

  optimizer_.setAlgorithm( solver );

  // Set terminate thresholds
  gainTerminateThreshold_ = 1e-6;

  // Set Convergence Criterion
//  g2o::SparseOptimizerTerminateAction* terminateAction = new g2o::SparseOptimizerTerminateAction();
//  terminateAction->setGainThreshold( gainTerminateThreshold_ );
//  optimizer_.addPostIterationAction( terminateAction );

}

CameraPose tracker_g2o::RefineCameraPose(
  const CameraPose& estimatedCameraPose,
  const std::map<MapPoint*, std::pair<Measurement, Measurement> >& measurementsStereo,
  const std::map<MapPoint*, Measurement>& measurementsLeft,
  const std::map<MapPoint*, Measurement>& measurementsRight
)
{

  size_t numStereoMeas = measurementsStereo.size();

  // We need at least 4 measurements to get a solution.
  // Make sure there are a bunch more just to be robust.
  if( numStereoMeas < 10 ) {
    std::cerr << std::endl << std::endl << "WARNING: Not enough points for tracking." << std::endl << std::endl;
    return estimatedCameraPose;
  }


  // freeing the graph memory
  optimizer_.clear();

  int nKF_BundleID = 0; // keyframe index for g2o

  auto cam_vertex = BuildNewVertex(nKF_BundleID, estimatedCameraPose, false);

  optimizer_.addVertex( cam_vertex );

  // Add the points' 3D position

  int nPoint_BundleID = 1;

  for( auto& mapPoint_meas_meas : measurementsStereo ) {

    MapPoint* mapPoint = mapPoint_meas_meas.first;

    auto point_vertex = BuildNewVertex(nPoint_BundleID, *mapPoint);

    optimizer_.addVertex( point_vertex );

    Measurement measurementLeft = mapPoint_meas_meas.second.first;

    Measurement measurementRight = mapPoint_meas_meas.second.second;

    auto edge = BuildNewStereoEdge(nPoint_BundleID, nKF_BundleID, measurementLeft.GetProjection(), measurementRight.GetProjection());

    optimizer_.addEdge( edge );

    nPoint_BundleID++;
  }

  for( auto& mapPoint_meas : measurementsLeft ) {

    MapPoint* mapPoint = mapPoint_meas.first;

    auto point_vertex = BuildNewVertex(nPoint_BundleID, *mapPoint);

    optimizer_.addVertex( point_vertex );

    Measurement measurement = mapPoint_meas.second;

    auto edge = BuildNewMonoEdge(nPoint_BundleID, nKF_BundleID, measurement.GetProjection());

    optimizer_.addEdge( edge );

    nPoint_BundleID++;
  }

  // Right measurements (alone) are not working so great
  //  for( auto& mapPoint_meas : measurementsRight ) {

  //    MapPoint* mapPoint = mapPoint_meas.first;

  //    auto point_vertex = BuildNewVertex(nPoint_BundleID, *mapPoint);

  //    optimizer_.addVertex( point_vertex );

  //    Measurement measurement = mapPoint_meas.second;

  //    auto edge = BuildNewMonoEdgeRight(nPoint_BundleID, nKF_BundleID, measurement.GetProjection());

  //    optimizer_.addEdge( edge );

  //    nPoint_BundleID++;
  //  }

  optimizer_.initializeOptimization();

  // TODO: pass as parameter
  optimizer_.optimize( 10 );

  return GetPose( nKF_BundleID );
}

// Attention: This changed with repect to the BundleDriver class
g2o::OptimizableGraph::Vertex* tracker_g2o::BuildNewVertex(int vertex_id, const CameraPose& cameraPose, const bool isFixed)
{
  // the position and the orientation (as quaternion is computed) in the expected format of g2o
  cv::Vec4d orientation_cv = cameraPose.GetOrientationQuaternion();
  cv::Vec3d position_cv = cameraPose.GetPosition();

  Eigen::Vector3d position_eigen = cv2eigen( position_cv );
  Eigen::Quaterniond orientation_eigen = cv2eigen( orientation_cv );

  // set up initial camera estimate
  g2o::SBACam sbacam(orientation_eigen, position_eigen);
  sbacam.setKcam(
    focal_length_[0], focal_length_[1],
    principal_point_[0], principal_point_[1],
    stereo_baseline_
  );

  g2o::VertexCam* v_se3 = new g2o::VertexCam();

  v_se3->setId( vertex_id );
  v_se3->setEstimate( sbacam );
  v_se3->setFixed( isFixed );

  return v_se3;
}

g2o::OptimizableGraph::Vertex* tracker_g2o::BuildNewVertex(int vertex_id, const MapPoint& mapPoint)
{
  cv::Point3d point_cv = mapPoint.GetPosition();
  Eigen::Vector3d point = cv2eigen( point_cv );

  g2o::VertexSBAPointXYZ* v_p = new g2o::VertexSBAPointXYZ();

  v_p->setId( vertex_id );
  v_p->setMarginalized( false ); // Attention: This changed with repect to the BundleDriver class
  v_p->setEstimate( point );
  v_p ->setFixed( true );

  return v_p;
}


g2o::OptimizableGraph::Edge* tracker_g2o::BuildNewMonoEdge(
  int pointId, int keyFrameId, const cv::Point2d& projection
)
{
  Eigen::Vector2d z(projection.x, projection.y);

  g2o::EdgeProjectP2MC * e = new g2o::EdgeProjectP2MC();
  e->vertices()[0]
      = dynamic_cast<g2o::OptimizableGraph::Vertex*>
      (optimizer_.vertices().find(pointId)->second);

  e->vertices()[1]
      = dynamic_cast<g2o::OptimizableGraph::Vertex*>
      (optimizer_.vertices().find(keyFrameId)->second);

  e->setMeasurement(z);

  e->information() = Eigen::Matrix2d::Identity();

  g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;

  e->setRobustKernel(rk);

  return e;
}


g2o::OptimizableGraph::Edge* tracker_g2o::BuildNewMonoEdgeRight(
  int pointId, int keyFrameId, const cv::Point2d& projection
)
{
  Eigen::Vector2d z(projection.x, projection.y);

  g2o::EdgeProjectP2MCRight * e = new g2o::EdgeProjectP2MCRight();
  e->vertices()[0]
      = dynamic_cast<g2o::OptimizableGraph::Vertex*>
      (optimizer_.vertices().find(pointId)->second);

  e->vertices()[1]
      = dynamic_cast<g2o::OptimizableGraph::Vertex*>
      (optimizer_.vertices().find(keyFrameId)->second);

  e->setMeasurement(z);

  e->information() = Eigen::Matrix2d::Identity();

  g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;

  e->setRobustKernel(rk);

  return e;
}

g2o::OptimizableGraph::Edge* tracker_g2o::BuildNewStereoEdge(
  int pointId, int keyFrameId, const cv::Point2d& projectionLeft, const cv::Point2d& projectionRight
)
{
  Eigen::Vector3d z(projectionLeft.x , projectionLeft.y, projectionRight.x);

  g2o::EdgeProjectP2SC *e = new g2o::EdgeProjectP2SC();

  e->vertices()[0]
      = dynamic_cast<g2o::OptimizableGraph::Vertex*>
      ( optimizer_.vertices().find(pointId)->second );

  e->vertices()[1]
      = dynamic_cast<g2o::OptimizableGraph::Vertex*>
      ( optimizer_.vertices().find(keyFrameId)->second );

  e->setMeasurement(z);

  e->information() = Eigen::Matrix3d::Identity();

  g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
  e->setRobustKernel(rk);

  return e;
}

CameraPose tracker_g2o::GetPose( int keyFrameId )
{
  g2o::HyperGraph::VertexIDMap::iterator v_it = optimizer_.vertices().find( keyFrameId );
  g2o::VertexCam *v_c = dynamic_cast< g2o::VertexCam * > (v_it->second);
  Eigen::Vector3d position = v_c->estimate().translation();
  g2o::Quaterniond qOrientation = v_c->estimate().rotation();

  Eigen::Matrix3d rotation = (qOrientation.toRotationMatrix()).transpose();

  Eigen::Vector3d translation = -rotation * position; // t = -R * C where C is the camera position
  cv::Vec3d cvTranslation(translation[0], translation[1], translation[2]);
  cv::Mat cvRotation(3,3,CV_64FC1);
  eigen2cv(rotation, cvRotation);

  return CameraPose(cvTranslation, cvRotation);
}
