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

#include "BundleDriver.hpp"
#include "thirdparty/MEstimator.hpp"
#include "utils/projective_math.hpp"
#include "utils/macros.hpp"
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

#ifdef SHOW_TRACKED_FRAMES
  #include "utils/Draw.hpp"
#endif


BundleDriver::BundleDriver(
  const cv::Matx33d& intrinsicLeft,
  const cv::Matx33d& intrinsicRight,
  const double stereo_baseline)
  : bundleAbortRequested_( false ), bundleAbortRequestedCopy_( false )
{
  focal_length_ = cv2eigen( getFocalLength( intrinsicLeft ) );
  principal_point_ = cv2eigen( getPrincipalPoint( intrinsicLeft ) );

  // stereo baseline (must be a possitive value)
  baseline_ = std::abs( stereo_baseline );

    /// SETUP G2O ///

  optimizer_.setVerbose( false );

  optimizer_.setForceStopFlag( &bundleAbortRequested_ );

  // Higher converge velocity
  g2o::BlockSolver_6_3::LinearSolverType* linearSolver
    = new g2o::LinearSolverPCG<g2o::BlockSolver_6_3::PoseMatrixType>();

  // Higher confident (better than CHOLMOD see paper: "3-D MappingWith an RGB-D Camera")
//  g2o::BlockSolver_6_3::LinearSolverType* linearSolver
//    = new g2o::LinearSolverCSparse<g2o::BlockSolver_6_3::PoseMatrixType>();

  // Higher confident
//  g2o::BlockSolver_6_3::LinearSolverType* linearSolver
//    = new g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>();

  g2o::BlockSolver_6_3* solver_ptr = new g2o::BlockSolver_6_3( linearSolver );

  // choosing the method to use by optimizer
  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( solver_ptr );

  optimizer_.setAlgorithm(solver);

  // Set terminate thresholds
  gainTerminateThreshold_ = 1e-6;

  // Set Convergence Criterion
  g2o::SparseOptimizerTerminateAction* terminateAction = new g2o::SparseOptimizerTerminateAction();
  terminateAction->setGainThreshold(gainTerminateThreshold_);
  optimizer_.addPostIterationAction(terminateAction);
}

void BundleDriver::SetData(const std::vector<StereoFrame*>& views, const std::vector<MapPoint*>& points)
{
  // clear all previous information
  Clear();

  // Id pool for the vertices
  int vertex_id = 0;

  // Add the non-fixed keyframes' poses to the bundle adjuster.
  for ( auto& keyFrame : views ) {

    // update id maps
    id_maps_.view_to_BundleId[ keyFrame ] = vertex_id;
    id_maps_.BundleId_to_view[ vertex_id ] = keyFrame;

    auto vertex = BuildNewVertex( vertex_id, keyFrame->GetCameraPose(), keyFrame->isFixed() );

    optimizer_.addVertex( vertex );

    vertex_id++;
  }

  // Add the points' 3D position
  for ( auto& mapPoint : points ) {

    // update id maps
    id_maps_.point_to_BundleId[ mapPoint ] = vertex_id;
    id_maps_.BundleId_to_point[ vertex_id ] = mapPoint;

    auto vertex = BuildNewVertex( vertex_id, *mapPoint, true, false);

    optimizer_.addVertex( vertex );

    vertex_id++;
  }

  // Add the relevant point-in-keyframe measurements
  AddMeasToBundle();
}

void BundleDriver::SetData(
  const std::set<StereoFrame*>& adjustViews,
  const std::set<StereoFrame*>& fixedViews,
  const std::set<MapPoint*>& points)
{
  // clear all previous information
  Clear();

  // Id pool for the vertices
  int vertex_id = 0;

  // Add the non-fixed keyframes' poses to the bundle adjuster.
  for ( auto& keyFrame : adjustViews ) {

    // update id maps
    id_maps_.view_to_BundleId[ keyFrame ] = vertex_id;
    id_maps_.BundleId_to_view[ vertex_id ] = keyFrame;

    auto vertex = BuildNewVertex( vertex_id, keyFrame->GetCameraPose(), false );

    optimizer_.addVertex( vertex );

    vertex_id++;
  }

  // Add the fixed keyframes' poses to the bundle adjuster.
  for ( auto& keyFrame : fixedViews ) {

    // update id maps
    id_maps_.view_to_BundleId[ keyFrame ] = vertex_id;
    id_maps_.BundleId_to_view[ vertex_id ] = keyFrame;

    auto vertex = BuildNewVertex( vertex_id, keyFrame->GetCameraPose(), true );

    optimizer_.addVertex( vertex );

    vertex_id++;
  }

  // Add the points' 3D position
  for ( auto& mapPoint : points ) {

    // update id maps
    id_maps_.point_to_BundleId[ mapPoint ] = vertex_id;
    id_maps_.BundleId_to_point[ vertex_id ] = mapPoint;

    auto vertex = BuildNewVertex( vertex_id, *mapPoint, true, false );

    optimizer_.addVertex( vertex );

    vertex_id++;
  }

  // Add the relevant point-in-keyframe measurements
  AddMeasToBundle();

}

void BundleDriver::AddMeasToBundle()
{

  for ( auto& view_id_map : id_maps_.view_to_BundleId ) {

    StereoFrame* keyFrame = view_id_map.first;
    int nKF_BundleID = id_maps_.view_to_BundleId.at( keyFrame );

    // Remove measurements of points which they were already removed
    // this happens when measurements are created during the tracking
    // but before the keyframe is added to the Mapper, the mapping has already removed the point
    keyFrame->RemoveBadPointMeasurements();

    for ( auto& point_stereo_meas_map : keyFrame->GetMeasurementsStereo() ) {

      MapPoint* mapPoint = point_stereo_meas_map.first;

      // this is necessary for the points which have been seen by the fix keyframe but their were not seen by the others keyframes
      if( not id_maps_.point_to_BundleId.count( mapPoint ) )
        continue;

      const Measurement& measurementLeft = point_stereo_meas_map.second.first;
      const Measurement& measurementRight = point_stereo_meas_map.second.second;

      int nPoint_BundleID = id_maps_.point_to_BundleId.at( mapPoint );

      auto e = BuildNewStereoEdge(nPoint_BundleID, nKF_BundleID, measurementLeft.GetProjection(), measurementRight.GetProjection());

      optimizer_.addEdge( e );
    }

    for ( auto& point_meas_map : keyFrame->GetMeasurementsLeftOnly() ) {

      MapPoint* mapPoint = point_meas_map.first;

      // this is necessary for the points which have been seen by the fix keyframe but their were not seen by the others keyframes
      if( not id_maps_.point_to_BundleId.count( mapPoint ) )
        continue;

      const Measurement& measurement = point_meas_map.second;

      int nPoint_BundleID = id_maps_.point_to_BundleId.at( mapPoint );

      auto e = BuildNewMonoEdge(nPoint_BundleID, nKF_BundleID, measurement.GetProjection());

      optimizer_.addEdge( e );
    }

    // TODO: Right measurements (alone) are not worked so good
//    for ( auto& point_meas_map : keyFrame->GetMeasurementsRightOnly() ) {

//      MapPoint* mapPoint = point_meas_map.first;

//      // this is necessary for the points which have been seen by the fix keyframe but their were not seen by the others keyframes
//      if( not id_maps_.point_to_BundleId.count( mapPoint ) ) {
//        continue;
//      }

//      const Measurement& measurement = point_meas_map.second;

//      int nPoint_BundleID = id_maps_.point_to_BundleId.at( mapPoint );

//      auto e = BuildNewMonoEdgeRight(nPoint_BundleID, nKF_BundleID, measurement.GetProjection());

//      optimizer_.addEdge( e );
//    }
  }
}


bool BundleDriver::Adjust(int maxIterations)
{
  optimizer_.initializeOptimization();

  // TODO: pass as argument
  optimizer_.optimize( maxIterations );

  // set abortRequest flag false (TODO: I am not sure if it goes here) -> I think not =) <--- discussion between developers
  bool aborted = bundleAbortRequestedCopy_;
  bundleAbortRequested_ = false;
  bundleAbortRequestedCopy_ = bundleAbortRequested_;

  return !aborted; // aborted = false, occurs when the method converge or when it hits the max iteration number
}

void BundleDriver::SavePoints()
{
  // The points are updated after the optimization
  for ( auto& point_id_map : id_maps_.point_to_BundleId ) {
    MapPoint* mapPoint = point_id_map.first;
    mapPoint->updatePosition( GetPoint( point_id_map.second ) );
  }
}

void BundleDriver::SaveCameras()
{
  // The camera poses are updated after the optimization
  for ( auto& view_id_map : id_maps_.view_to_BundleId ) {
    StereoFrame* keyFrame = view_id_map.first;
    keyFrame->UpdateCameraPose( GetPose( view_id_map.second ) );
  }
}

std::list< std::pair<StereoFrame*, MapPoint*> > BundleDriver::GetBadMeasurements()
{
  const g2o::SparseOptimizer::EdgeContainer& activeEdges = optimizer_.activeEdges();

  // convert to std::vector<double>
  std::vector<double> errorSquareds;
  errorSquareds.reserve( activeEdges.size() );
  for (auto edge : activeEdges)
    errorSquareds.push_back( edge->chi2() );

  // Compute the error distribution
  double dSigmaSquared = Tukey::FindSigmaSquared( errorSquareds );

  std::list< std::pair<StereoFrame*, MapPoint*> > badMeasurements;

  for ( auto& edge_ptr : activeEdges ) {

    // compute weight
    double dWeight = Tukey::Weight(edge_ptr->chi2(), dSigmaSquared);

    if( dWeight == 0.0) {

      const g2o::HyperGraph::VertexContainer& vertices = edge_ptr->vertices();

      const g2o::HyperGraph::Vertex *v_p = vertices[0]; // take the point vertex
      const g2o::HyperGraph::Vertex *v_c = vertices[1]; // take the camera vertex

      MapPoint* pp = id_maps_.BundleId_to_point[ v_p->id() ];
      StereoFrame* pk = id_maps_.BundleId_to_view[ v_c->id() ];

      badMeasurements.push_back(std::pair<StereoFrame*, MapPoint*>(pk,pp));
    }
  }

  // Debugging
//  for ( auto& view_id_map : id_maps_.view_to_BundleId ) {
//    StereoFrame* keyFrame = view_id_map.first;
//    std::cout << "kf: " << keyFrame->GetId() << " mediciones izq: " << keyFrame->GetMeasurementsLeft().size() << std::endl;
//    std::cout << "kf: " << keyFrame->GetId() << " mediciones der: " << keyFrame->GetMeasurementsRight().size() << std::endl;
//  }

  return badMeasurements;
}

void BundleDriver::Clear()
{
  // freeing the graph memory
  optimizer_.clear();

  id_maps_ = IdMaps();
}

void BundleDriver::Break()
{
  bundleAbortRequested_ = true;
  bundleAbortRequestedCopy_ = bundleAbortRequested_;
}

g2o::OptimizableGraph::Vertex* BundleDriver::BuildNewVertex(
  int vertex_id, const CameraPose& cameraPose, const bool isFixed)
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
    baseline_
  );

  g2o::VertexCam* v_se3 = new g2o::VertexCam();

  v_se3->setId( vertex_id );
  v_se3->setEstimate( sbacam );
  v_se3->setFixed( isFixed );

  return v_se3;
}

g2o::OptimizableGraph::Vertex* BundleDriver::BuildNewVertex(int vertex_id, const MapPoint& mapPoint, const bool marginalize ,const bool isFixed)
{
  cv::Point3d point_cv = mapPoint.GetPosition();
  Eigen::Vector3d point = cv2eigen( point_cv );

  g2o::VertexSBAPointXYZ* v_p = new g2o::VertexSBAPointXYZ();

  v_p->setId( vertex_id );
  v_p->setMarginalized( marginalize );
  v_p->setEstimate( point );
  v_p ->setFixed( isFixed );

  return v_p;
}

g2o::OptimizableGraph::Edge* BundleDriver::BuildNewMonoEdge(
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


g2o::OptimizableGraph::Edge* BundleDriver::BuildNewMonoEdgeRight(
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


g2o::OptimizableGraph::Edge* BundleDriver::BuildNewStereoEdge(
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

cv::Vec3d BundleDriver::GetPoint( int pointId )
{
  g2o::HyperGraph::VertexIDMap::iterator v_it = optimizer_.vertices().find( pointId );
  g2o::VertexSBAPointXYZ * v_p = dynamic_cast< g2o::VertexSBAPointXYZ * > (v_it->second);
  Eigen::Vector3d point = v_p->estimate();
  return cv::Vec3d(point[0], point[1], point[2]);
}

CameraPose BundleDriver::GetPose( int keyFrameId )
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
