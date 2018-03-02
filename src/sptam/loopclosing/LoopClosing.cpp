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

#include <list>
#include <tuple>

#include "../Map.hpp"
#include "LoopClosing.hpp"
#include "PoseEstimator.hpp"
#include "SmoothEstimatePropagator.hpp"
#include "StereoMatcher.hpp"

#include "../MapMakerThread.hpp"
#include "../sptam.hpp"

#include "opencv2/features2d/features2d.hpp"

// Eigen
#include <Eigen/Core>
#include <Eigen/Geometry>

// G2O
#include <g2o/config.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>

#ifdef SHOW_PROFILING
  #include "../utils/log/Profiler.hpp"
  #include "../utils/log/Logger.hpp"
#endif // SHOW_PROFILING

using namespace std;

LoopClosing::LoopClosing(SPTAM& tracker, MapMakerThread& mapper, sptam::Map& map, std::unique_ptr<LCDetector>& detector, const Parameters& params)
  : tracker_(tracker)
  , mapper_(mapper), map_(map)
  , loop_detector_(std::move(detector))
  , params_(params)
{
  maintenanceThread_ = std::thread(&LoopClosing::maintenance, this);
}

LoopClosing::~LoopClosing()
{}

void LoopClosing::addKeyFrame(sptam::Map::SharedKeyFrame& keyframe)
{
  // push new keyframe to queue and signal
  keyFrameQueue_.push( keyframe );

  #ifdef SHOW_PROFILING
    WriteToLog(" lc queueSize: ", keyFrameQueue_.size());
  #endif
}

void LoopClosing::addKeyFrames(const std::list<sptam::Map::SharedKeyFrame>& keyFrames)
{
  for(auto keyframe : keyFrames)
    keyFrameQueue_.push(keyframe);

  #ifdef SHOW_PROFILING
    WriteToLog(" lc queueSize: ", keyFrameQueue_.size());
  #endif
}

void matchAndEstimate(cv::Ptr<cv::DescriptorMatcher>& matcher,
                      double& matchingDistance,
                      const sptam::Map::SharedKeyFrame& query_keyframe,
                      const sptam::Map::SharedKeyFrame& match_keyframe,
                      size_t& nMatches, size_t& inliers, cv::Matx44d& estimated_pose,
                      vector<sptam::Map::SharedMeas>& query_stereo_measurements,
                      vector<sptam::Map::SharedMeas>& match_stereo_measurement,
                      vector<SDMatch>& stereo_matches)
{
  cv::Mat descriptors1;
  cv::Mat descriptors2;
  vector<cv::KeyPoint> kps1;
  vector<cv::KeyPoint> kps2;
  vector<cv::DMatch> matches12;

  uint i = 0;

  for(auto& measurement : query_keyframe->measurements())
    if(measurement->GetType() == Measurement::STEREO)
    {
      kps1.push_back(measurement->GetKeypoints()[0]);
      kps2.push_back(measurement->GetKeypoints()[1]);

      descriptors1.push_back(measurement->GetDescriptors()[0]);
      descriptors2.push_back(measurement->GetDescriptors()[1]);

      matches12.push_back(cv::DMatch(i,i,0));

      query_stereo_measurements.push_back(measurement);

      i++;
    }

  cv::Mat descriptors3;
  cv::Mat descriptors4;
  vector<cv::KeyPoint> kps3;
  vector<cv::KeyPoint> kps4;
  vector<cv::DMatch> matches34;

  i = 0;

  for(auto& measurement : match_keyframe->measurements())
    if(measurement->GetType() == Measurement::STEREO)
    {
      kps3.push_back(measurement->GetKeypoints()[0]);
      kps4.push_back(measurement->GetKeypoints()[1]);

      descriptors3.push_back(measurement->GetDescriptors()[0]);
      descriptors4.push_back(measurement->GetDescriptors()[1]);

      matches34.push_back(cv::DMatch(i,i,0));

      match_stereo_measurement.push_back(measurement);

      i++;
    }

  StereoMatcher::match(*matcher, matchingDistance,
                       descriptors1, descriptors2, matches12,
                       descriptors3, descriptors4, matches34, stereo_matches);

  /*cv::Mat outstereo;
  StereoMatcher::drawStereoMatches(query_keyframe, match_keyframe, stereo_matches, outstereo);
  cv::resize(outstereo, outstereo, cv::Size(), 0.45, 0.45); // resize to X%
  cv::namedWindow("Stereo matches"); cv::imshow("Stereo matches", outstereo);
  cv::waitKey(1);*/

  nMatches = stereo_matches.size();

  inliers = 0;

  #ifdef SHOW_PROFILING
  WriteToLog(" lc peMatches: ", nMatches);
  #endif

  // PoseEstimator needs at least 4 points for the minimal case.
  if(nMatches < 4)
    return;

  PoseEstimator pe(PoseEstimator::PETYPE::CENTRAL, PoseEstimator::MINIMAL_ALGORITHM::KNEIP,
                     PoseEstimator::GENERIC_ALGORITHM::UPNP, true);

  double focal_length = match_keyframe->GetFrameLeft().GetCamera().GetIntrinsics()(0,0);
  pe.setRansacPixelThreshold(1, focal_length);

  inliers = pe.estimatePose(query_keyframe, match_keyframe, kps1, kps2, kps3, kps4, stereo_matches, estimated_pose);
}

g2o::VertexSE3* addVertex(g2o::SparseOptimizer& optimizer, const unsigned int& currentFrameIndex, const Eigen::Isometry3d& pose, bool fixed){
  g2o::VertexSE3* v_se3 = new g2o::VertexSE3;

  v_se3->setId(currentFrameIndex);
  v_se3->setEstimate(pose);
  v_se3->setFixed(fixed);

  optimizer.addVertex(v_se3);

  return v_se3;
}

void configPoseGraph(const list<sptam::Map::SharedKeyFrame>& keyframes, const sptam::Map::SharedKeyFrame& queryKF, const sptam::Map::SharedKeyFrame& matchKF,
                     const cv::Matx44d& match_pose, std::aligned_vector<std::tuple<size_t, size_t, Eigen::Isometry3d>>& loop_frames,
                     g2o::SparseOptimizer& optimizer)
{
  optimizer.setVerbose(false);

  std::unique_ptr<g2o::BlockSolver_6_3::LinearSolverType> linearSolver( new g2o::LinearSolverCholmod<g2o::BlockSolver_6_3::PoseMatrixType>() );

  std::unique_ptr<g2o::BlockSolver_6_3> solver_ptr(new g2o::BlockSolver_6_3( std::move(linearSolver) ) );


  g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg( std::move(solver_ptr) );
  optimizer.setAlgorithm(solver);

  /* The uncertainty matrix: Represents the inverse covariance of the measurement, and thus is symmetric
   * and positive definite. */
  Eigen::Matrix<double,6,6> uncertainty = Eigen::Matrix<double,6,6>::Identity();

  g2o::VertexSE3* currentVertex = nullptr;
  g2o::VertexSE3* previousVertex = currentVertex;

  auto loops_it = loop_frames.begin();

  for(auto it = keyframes.begin(); it != keyframes.end(); it++){
    const sptam::Map::SharedKeyFrame& keyframe = *it;

    Eigen::Isometry3d keyframe_pose = Eigen::Isometry3d::Identity();
    keyframe_pose.linear() = keyframe->GetCameraPose().GetOrientationMatrix();
    keyframe_pose.translation() = keyframe->GetCameraPose().GetPosition();

    if(it == keyframes.begin())
      currentVertex = addVertex(optimizer, keyframe->GetId(), keyframe_pose, true);
    else
      currentVertex = addVertex(optimizer, keyframe->GetId(), keyframe_pose, false);

    /* Adding edges between sucesives keyframes of the map */
    if(it != keyframes.begin()) {
      // Pose of current in previous's reference frame, t = Tpc (takes points in current's reference and returns it in previous's reference)
      Eigen::Isometry3d t = previousVertex->estimate().inverse() * currentVertex->estimate();
      g2o::EdgeSE3* e = new g2o::EdgeSE3;
      e->setVertex(0, previousVertex);
      e->setVertex(1, currentVertex);
      e->setMeasurement(t);
      e->information() = uncertainty;
      optimizer.addEdge(e);
     }

    /* Adding edges of past loops detected */
    if(loops_it != loop_frames.end() && (size_t) keyframe->GetId() == get<0>(*loops_it)){
      g2o::VertexSE3* pastLoopVertex = static_cast<g2o::VertexSE3*>(optimizer.vertex(get<1>(*loops_it)));
      g2o::EdgeSE3* e = new g2o::EdgeSE3;

      e->setVertex(0, currentVertex);
      e->setVertex(1, pastLoopVertex);

      //pastLoopVertex->setFixed(true);

      /* Relative transformation between the vertices involved on this loop detected */
      e->setMeasurement(get<2>(*loops_it));

      e->information() = uncertainty;
      optimizer.addEdge(e);

      loops_it++;
    }

    previousVertex = currentVertex; // G2O vertex update
  }

  g2o::VertexSE3* currVertex = static_cast<g2o::VertexSE3*>(optimizer.vertex(queryKF->GetId()));
  g2o::VertexSE3* loopVertex = static_cast<g2o::VertexSE3*>(optimizer.vertex(matchKF->GetId()));

  /* Relative transformation between current vertex and loop vertex using openGV estimation */
  Eigen::Matrix<double,4,4> Tcl;
  cv::cv2eigen(match_pose, Tcl); // Tcl = loopPose = Twl (converting loop vertex estimated pose to Eigen)

  /* Tcl = Twc^-1 * Twl = currentPose^-1 * loopPose = Tcw * Twl = Tcl */
  Tcl = currVertex->estimate().inverse() * Tcl;

  g2o::EdgeSE3* e = new g2o::EdgeSE3;

  e->setVertex(0, currVertex);
  e->setVertex(1, loopVertex);

  //loopVertex->setFixed(true);

  e->setMeasurement(Eigen::Isometry3d(Tcl));
  e->information() = uncertainty;
  optimizer.addEdge(e);

  loop_frames.push_back(make_tuple(queryKF->GetId(), matchKF->GetId(), Eigen::Isometry3d(Tcl)));

  #ifdef SHOW_PROFILING
  std::stringstream message;
  message << " lc RELATIVE_TRANSFORMATION:" << " " << queryKF->GetId() << " " << matchKF->GetId() << " "
    << printFullPrecision( Tcl(0,0) ) << " " << printFullPrecision( Tcl(0,1) ) << " " << printFullPrecision( Tcl(0,2) ) << " " << printFullPrecision( Tcl(0,3) ) << " "
    << printFullPrecision( Tcl(1,0) ) << " " << printFullPrecision( Tcl(1,1) ) << " " << printFullPrecision( Tcl(1,2) ) << " " << printFullPrecision( Tcl(1,3) ) << " "
    << printFullPrecision( Tcl(2,0) ) << " " << printFullPrecision( Tcl(2,1) ) << " " << printFullPrecision( Tcl(2,2) ) << " " << printFullPrecision( Tcl(2,3) ) << " "
    << std::endl;

  Logger::Write( message.str() );
  #endif
}

template<typename Iterator>
void updatePosesAndPoints(const Iterator begin, const Iterator end, const sptam::Map::SharedKeyFrameSet& lba_safe_window, const g2o::SparseOptimizer& optimizer)
{
  if(begin == end /*or keyframes.size() <= lba_safe_window.size()*/) // All map was selected as for local BA
    return;

  #ifdef SHOW_PROFILING
  size_t updated_KFs = 0; // Note that safe_window ids KFs are not consecutive!! this is just for having an idea what is inside
  std::list<size_t> l;
  if(lba_safe_window.begin() != lba_safe_window.end())
  { l.push_back((*lba_safe_window.begin())->GetId()); l.push_back((*std::prev(lba_safe_window.end()))->GetId()); }
  WriteToLog(" lc KFsUpdateRange: ", l);
  #endif

  for (auto keyframe = begin; keyframe != end; keyframe++){

    if(lba_safe_window.count(*keyframe)) // avoid keyframes being use by the local BA
      continue;

    const g2o::VertexSE3* corrected_keyframe = static_cast<const g2o::VertexSE3*>(optimizer.vertex((*keyframe)->GetId()));

    /* non corrected Twi */
    Eigen::Isometry3d non_correctedTwi = Eigen::Isometry3d::Identity();
    non_correctedTwi.linear() = (*keyframe)->GetCameraPose().GetOrientationMatrix();
    non_correctedTwi.translation() = (*keyframe)->GetCameraPose().GetPosition();

    for(sptam::Map::SharedMeas& meas : (*keyframe)->measurements()){
      if(meas->GetSource() == Measurement::SRC_TRIANGULATION){
        const sptam::Map::SharedPoint& mp = meas->mapPoint();

        Eigen::Vector3d position = mp->GetPosition();

        Eigen::Vector4d hmg_position(position(0),position(1),position(2),1);

        /* corrected map point = corrTwi * non_corrTiw * position, this way we first interpret the point in
         * reference of the non corrected camera and then, as the relative direction of the point must be the same in
         * reference of the corrected camera, we transform the point in worlds reference using the corrected Twi */
        hmg_position = corrected_keyframe->estimate() * (non_correctedTwi.inverse() * hmg_position); // correcting map point position

        mp->updatePosition(Eigen::Vector3d(hmg_position(0)/hmg_position(3), hmg_position(1)/hmg_position(3), hmg_position(2)/hmg_position(3)));
      }
    }

    Eigen::Quaterniond orientation(corrected_keyframe->estimate().linear());
    (*keyframe)->UpdateCameraPose(CameraPose(corrected_keyframe->estimate().translation(), orientation, Eigen::Matrix6d::Identity()));

    #ifdef SHOW_PROFILING
    updated_KFs++;
    #endif
  }

  #ifdef SHOW_PROFILING
  WriteToLog(" lc KFsUpdated: ", updated_KFs);
  #endif
}

template<typename Iterator>
void updatePosesAndPoints(Iterator end, unsigned int size, const Eigen::Isometry3d& T)
{
  if(size == 0)
    return;

  #ifdef SHOW_PROFILING
  size_t updated_KFs = 0;
  /*std::list<size_t> l; l.push_back((*begin)->GetId()); l.push_back((*(end-1))->GetId());
  WriteToLog(" lc KFsUpdateRange: ", l);*/
  #endif

  for (auto keyframe = end; size > 0; keyframe--, size--){
    Eigen::Isometry3d correctedTwi = Eigen::Isometry3d::Identity();
    correctedTwi.linear() = (*keyframe)->GetCameraPose().GetOrientationMatrix();
    correctedTwi.translation() = (*keyframe)->GetCameraPose().GetPosition();

    correctedTwi = correctedTwi * T;

    Eigen::Isometry3d non_correctedTiw = Eigen::Isometry3d::Identity();
    non_correctedTiw.linear() = (*keyframe)->GetCameraPose().GetOrientationMatrix();
    non_correctedTiw.translation() = (*keyframe)->GetCameraPose().GetPosition();

    non_correctedTiw = non_correctedTiw.inverse();

    for(auto& meas : (*keyframe)->measurements()){
      if(meas->GetSource() == Measurement::SRC_TRIANGULATION){
        const sptam::Map::SharedPoint& mp = meas->mapPoint();

        Eigen::Vector3d position = mp->GetPosition();

        Eigen::Vector4d hmg_position(position(0),position(1),position(2),1);

        /* corrected map point = corrTwi * non_corrTiw * position, this way we first interpret the point in
         * reference of the non corrected camera and then, as the relative direction of the point must be the same in
         * reference of the corrected camera, we transform the point in worlds reference using the corrected Twi */
        hmg_position = correctedTwi * (non_correctedTiw * hmg_position); // correcting map point position

        mp->updatePosition(Eigen::Vector3d(hmg_position(0)/hmg_position(3), hmg_position(1)/hmg_position(3), hmg_position(2)/hmg_position(3)));
      }
    }

    Eigen::Quaterniond orientation(correctedTwi.linear());
    (*keyframe)->UpdateCameraPose(CameraPose(correctedTwi.translation(), orientation, Eigen::Matrix6d::Identity()));

    #ifdef SHOW_PROFILING
    updated_KFs++;
    #endif
  }

  #ifdef SHOW_PROFILING
  WriteToLog(" lc KFsUpdateFix: ", updated_KFs);
  #endif
}

void LoopClosing::maintenance()
{

  /* Gaston: Hack to ensure that indices handled by detector correspond
   * with a vectors index for quick access.*/
  vector<sptam::Map::SharedKeyFrame> processed_keyframes;

  while ( not stop_ ) {
    sptam::Map::SharedKeyFrame keyframe;

    keyFrameQueue_.waitAndPop( keyframe );

    if(stop_) // keyframeQueue could've been awaken by a stop.
      return;

    /* TODO: Improve keyframes database for consistent and quick retrieval */
    processed_keyframes.push_back(keyframe);

    DetectionMatch dm;

    #ifdef SHOW_PROFILING
      sptam::Timer t_detection;
      t_detection.start();
    #endif

    dm = loop_detector_->detectloop(keyframe);

    #ifdef SHOW_PROFILING
      t_detection.stop();
      WriteToLog(" lc detection: ", t_detection);
    #endif

    if(not dm.detection())
      continue;

    /* Mapping wont be paused at this stage, instead we will optimize the keyframes until this moment and
     * apply a rigid transformation for any other that gets createad afterwards */

    size_t nMatches;
    cv::Matx44d match_pose;
    size_t inliers;
    sptam::Map::SharedKeyFrame& query_keyframe = processed_keyframes[dm.query];
    sptam::Map::SharedKeyFrame& match_keyframe = processed_keyframes[dm.match];
    Eigen::Vector3d query_position;

    vector<sptam::Map::SharedMeas> query_stereo_measurements;
    vector<sptam::Map::SharedMeas> match_stereo_measurements;
    vector<SDMatch> stereo_matches;

    #ifdef SHOW_PROFILING
      sptam::Timer t_matching_pe;
      t_matching_pe.start();
    #endif

    query_position = query_keyframe->GetPosition();

    // TODO: LC doesnt support KFs removal now, but when it does we need to tell Mapping that doesnt do it now

    matchAndEstimate(params_.descriptorMatcher, params_.matchingDistanceThreshold,
                     query_keyframe, match_keyframe, nMatches, inliers, match_pose,
                     query_stereo_measurements, match_stereo_measurements, stereo_matches);

    #ifdef SHOW_PROFILING
      t_matching_pe.stop();
      WriteToLog(" lc matching_pe: ", t_matching_pe);
    #endif

    if(not (inliers >= 20 and (double) inliers >= nMatches * 0.8)){
      #ifdef SHOW_PROFILING
      std::list<size_t> l; l.push_back(query_keyframe->GetId()); l.push_back(match_keyframe->GetId());
      l.push_back(nMatches); l.push_back(inliers);
      WriteToLog(" lc REJECTED_LOOP: ", l);
      #endif
      continue;
    }

    if(std::abs(query_position(0) - match_pose(0,3)) > 5 or std::abs(query_position(1) - match_pose(1,3)) > 5 or std::abs(query_position(2) - match_pose(2,3)) > 5){
      #ifdef SHOW_PROFILING
      std::list<size_t> l; l.push_back(query_keyframe->GetId()); l.push_back(match_keyframe->GetId());
      l.push_back(nMatches); l.push_back(inliers);
      WriteToLog(" lc REJECTED_LOOP_TOOFAR: ", l);
      #endif
      continue;
    }

    #ifdef SHOW_PROFILING
    std::list<size_t> l; l.push_back(query_keyframe->GetId()); l.push_back(match_keyframe->GetId());
    l.push_back(nMatches); l.push_back(inliers);
    WriteToLog(" lc ACCEPTED_LOOP: ", l);
    #endif

    /* We have to ensure that the mapping thread is on a safe part of code, before the selection of KFs to optimize */
    sptam::Map::SharedKeyFrameSet safe_window = mapper_.lockUsageWindow();

    safe_window.insert(tracker_.getReferenceKeyframe());
    for(auto& kframe : tracker_.getReferenceKeyframe()->covisibilityKeyFrames())
      safe_window.insert(kframe.first);

    g2o::SparseOptimizer optimizer;

    /* We use the Rt of the keyframes that may be in use by the Local BA before the closure
     * to calculate the optimization that lba has introduce while we where optimizing the graph (very time consuming) */
    std::aligned_vector<Eigen::Isometry3d> lba_kfs_before_lc;

    list<sptam::Map::SharedKeyFrame> considered_keyframes;

    /* The safe window established between the Local Mapping must be inside the considered KFs.
     * TODO: Keyframe removal needs to be handled in a way that doesnt invalidate this kind of lists */
    {
      boost::shared_lock<boost::shared_mutex> lock(map_.map_mutex_);
      considered_keyframes = map_.getKeyframes();
    }

    // Loading keyframes poses and previous loops restritions
    configPoseGraph(considered_keyframes, query_keyframe, match_keyframe, match_pose, loop_frames_, optimizer);

    for (auto it = safe_window.begin(); it != safe_window.end(); it++){
      Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
      pose.linear() = (*it)->GetCameraPose().GetOrientationMatrix();
      pose.translation() = (*it)->GetCameraPose().GetPosition();

      lba_kfs_before_lc.push_back(pose.inverse()); // Copyng of each Tcw (rotation,translation) of the keyframes on the lba zone
    }

    #ifdef SHOW_PROFILING
      sptam::Timer t_optimization;
      t_optimization.start();
    #endif

    g2o::VertexSE3* loopVertex = static_cast<g2o::VertexSE3*>(optimizer.vertex(match_keyframe->GetId()));

    /* Note: Printing the whole pose graph its very time consuming, activate this debug output only when complete map
     * visualization its needed. */
    #ifdef SHOW_PROFILING
    /*std::stringstream ss;
    ss.str(""); ss << "./optimizations/LC_pre_" << query_keyframe->GetId() << "_" << match_keyframe->GetId() << ".g2o";
    optimizer.save(ss.str().c_str());*/
    #endif

    optimizer.initializeOptimization();

    // Propagate initial estimate through 10% of total keyframes (or at least 20 keyframes)
    double maxDistance = std::max(20.0, considered_keyframes.size() * 0.1);

    SmoothEstimatePropagator sEPropagator(&optimizer, maxDistance);

    sEPropagator.propagate(loopVertex);

    #ifdef SHOW_PROFILING
    /*ss.str(""); ss << "./optimizations/LC_prop_" << query_keyframe->GetId() << "_" << match_keyframe->GetId() << ".g2o";
    optimizer.save(ss.str().c_str());*/
    #endif

    optimizer.optimize( 20 );

    #ifdef SHOW_PROFILING
      t_optimization.stop();
      WriteToLog(" lc optimization: ", t_optimization);
    #endif

    #ifdef SHOW_PROFILING
    /*ss.str(""); ss << "./optimizations/LC_optimized_" << query_keyframe->GetId() << "_" << match_keyframe->GetId() << ".g2o";
    optimizer.save(ss.str().c_str());*/
    #endif

    /* Updating keyframes and map points, we will first update those keyframes that arent being used by the mapper
     * Those keyframes are safe, no one are using them. Map points have internal locking, so there is no need for map lock */
    #ifdef SHOW_PROFILING
      sptam::Timer t_unsafe_update;
      t_unsafe_update.start();
    #endif

    // Exclude KFs that may being use by the local BA.
    updatePosesAndPoints(considered_keyframes.begin(), considered_keyframes.end(), safe_window, optimizer);

    #ifdef SHOW_PROFILING
      t_unsafe_update.stop();
      WriteToLog(" lc unsafe_map_update: ", t_unsafe_update);
    #endif


    tracker_.stopAddingKeyframes();

    #ifdef SHOW_PROFILING
    sptam::Timer t_mapping_pause;
    t_mapping_pause.start();
    #endif

    /* Wait until mapper flushes everything to the map */
    mapper_.waitUntilEmptyQueue();

    while(mapper_.isProcessing())
      std::this_thread::yield();

    #ifdef SHOW_PROFILING
    t_mapping_pause.stop();
    WriteToLog(" lc mapping_pause: ", t_mapping_pause);
    #endif

    /* Calculating optimization introduced by local mapping while loop was been closed */
    int i = 0;
    for(auto lba_keyframe : safe_window){

      Eigen::Isometry3d lba_kf_after_lc = Eigen::Isometry3d::Identity();
      lba_kf_after_lc.linear() = lba_keyframe->GetCameraPose().GetOrientationMatrix();
      lba_kf_after_lc.translation() = lba_keyframe->GetCameraPose().GetPosition();

      // Calculating inbetween optimization introduced by the LBA: Tcc' = Tcw * Twc'
      Eigen::Isometry3d lba_opt = lba_kfs_before_lc[i] * lba_kf_after_lc;

      g2o::VertexSE3* vertex = static_cast<g2o::VertexSE3*>(optimizer.vertex(lba_keyframe->GetId()));

      // Correcting the g2o optimization carried with the adjusts introduced by the lba while we where busy.
      vertex->setEstimate(vertex->estimate() * lba_opt);

      i++;
    }

    #ifdef SHOW_PROFILING
    sptam::Timer t_tracking_pause;
    t_tracking_pause.start();
    #endif

    tracker_.pause();

    #ifdef SHOW_PROFILING
    t_tracking_pause.stop();
    WriteToLog(" lc tracking_pause: ", t_tracking_pause);
    #endif

    #ifdef SHOW_PROFILING
      sptam::Timer t_safeupdate;
      t_safeupdate.start();
    #endif

    /* NOTE: Gaston: This is miss handling the Tracker's local map. Is taking
                     the lastest added keyframe as the most recent keyframe used by Tracker! */

    /* Estimated error after loop closure: T = Tc1w * Twc2 = Tc1c2 with c1 the last frame before optimization and
     * c2 the last frame after optimization.
     * NOTE: We need to ensure that no other thread modifies this relative transformation! thats why we do it after locking */
    Eigen::Isometry3d non_correctedTwc = Eigen::Isometry3d::Identity();
    non_correctedTwc.linear() = (*(std::prev(considered_keyframes.end())))->GetCameraPose().GetOrientationMatrix();
    non_correctedTwc.translation() = (*(std::prev(considered_keyframes.end())))->GetCameraPose().GetPosition();

    // Corrected pose of the last optimized keyframe
    g2o::VertexSE3* last_vertex = static_cast<g2o::VertexSE3*>(optimizer.vertex((*(std::prev(considered_keyframes.end())))->GetId()));

    Eigen::Isometry3d T = Eigen::Isometry3d(non_correctedTwc).inverse() * last_vertex->estimate();

    // We need to wait for the end of the current frame tracking and ensure that we wont interfere with the tracker.
    while(tracker_.isTracking())
      std::this_thread::yield();

    tracker_.setLoopCorrection(T);

    // Updating keyframes and map points on the lba zone
    updatePosesAndPoints(safe_window.begin(), safe_window.end(), sptam::Map::SharedKeyFrameSet(), optimizer);

    const list<sptam::Map::SharedKeyFrame>& after_lc_kfs = map_.getKeyframes();

    // Updating out of time keyframes created after the current loop closing procedure
    updatePosesAndPoints(after_lc_kfs.size() > 0 ? std::prev(after_lc_kfs.end()) : after_lc_kfs.end(), after_lc_kfs.size() - considered_keyframes.size(), T);

    #ifdef SHOW_PROFILING
      t_safeupdate.stop();
      WriteToLog(" lc safe_map_update: ", t_safeupdate);
    #endif

    for(auto meas_match : stereo_matches)
    {

      Measurement new_query_meas(Measurement::STEREO, Measurement::SRC_REFIND,
                                 query_stereo_measurements[meas_match.m1vs3.queryIdx]->GetKeypoints(),
                                 query_stereo_measurements[meas_match.m1vs3.queryIdx]->GetDescriptors());
      map_.addMeasurement(query_keyframe, match_stereo_measurements[meas_match.m1vs3.trainIdx]->mapPoint(),
                          new_query_meas);

      Measurement new_match_meas(Measurement::STEREO, Measurement::SRC_REFIND,
                                 match_stereo_measurements[meas_match.m1vs3.trainIdx]->GetKeypoints(),
                                 match_stereo_measurements[meas_match.m1vs3.trainIdx]->GetDescriptors());
      map_.addMeasurement(match_keyframe, query_stereo_measurements[meas_match.m1vs3.queryIdx]->mapPoint(),
                          new_match_meas);
    }

    mapper_.freeUsageWindow();
    tracker_.resumeAddingKeyframes();
    tracker_.unPause();

    keyFrameQueue_.clear(); // forget too recent keyframes
  }
}
