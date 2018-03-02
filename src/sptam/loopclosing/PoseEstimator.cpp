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

#include "PoseEstimator.hpp"

#include <Eigen/Eigen>

// OpenCV
#include <opencv2/core/core.hpp>
#include <opencv2/core/eigen.hpp>

using namespace std;
using namespace cv;
using namespace opengv;

PoseEstimator::PoseEstimator(const PETYPE& type, const MINIMAL_ALGORITHM& minmet,
                             const GENERIC_ALGORITHM& genmet, const bool& optimization)
  : pe_type(type), minimal_method(minmet), generic_method(genmet), nonlinear_optimization(optimization)
{
  ransac.threshold_ = (1.0 - cos(atan(0.5/800))); // default focal length of 800 and 0.5 pixel reprojection tolerance
  ransac.max_iterations_ = 50; // default 50 ransac iterations
}

void PoseEstimator::setRansacThreshold(const double& t){ransac.threshold_ = t;}

// reprojection threshold expreses as pixels tolerance, refer to OpenGV paper for more information.
void PoseEstimator::setRansacPixelThreshold(const double& pixels, const double& focal_length)
{ ransac.threshold_ = (1.0 - cos(atan(pixels/focal_length))); }

void PoseEstimator::setRansacIterations(const int& i){ ransac.max_iterations_ = i; }

size_t PoseEstimator::estimatePose(const sptam::Map::SharedKeyFrame& referenceFrame, const sptam::Map::SharedKeyFrame& targetFrame, const std::vector<SDMatch>& stereo_matches, Matx44d& pose)
{
  return estimatePose(referenceFrame, targetFrame,
                      referenceFrame->GetFrameLeft().GetFeatures().GetKeypoints(), referenceFrame->GetFrameRight().GetFeatures().GetKeypoints(),
                      targetFrame->GetFrameLeft().GetFeatures().GetKeypoints(), targetFrame->GetFrameRight().GetFeatures().GetKeypoints(),
                      stereo_matches, pose);
}

size_t PoseEstimator::estimatePose(const sptam::Map::SharedKeyFrame& referenceFrame, const sptam::Map::SharedKeyFrame& targetFrame,
                    const std::vector<cv::KeyPoint>& kps1, const std::vector<cv::KeyPoint>& kps2,
                    const std::vector<cv::KeyPoint>& kps3, const std::vector<cv::KeyPoint>& kps4,
                    const std::vector<SDMatch>& stereo_matches, cv::Matx44d& pose)
{
  switch(pe_type) {
    case PETYPE::CENTRAL: return centralPoseEstimation(referenceFrame, targetFrame,
                                                       kps1, kps2,
                                                       kps3,
                                                       stereo_matches, pose);
    //case PETYPE::NONCENTRAL: return noncentralPoseEstimation(referenceFrame, targetFrame, stereo_matches);
    default: return centralPoseEstimation(referenceFrame, targetFrame,
                                          kps1, kps2,
                                          kps3,
                                          stereo_matches, pose);
  }
}

size_t PoseEstimator::centralPoseEstimation(const sptam::Map::SharedKeyFrame& referenceFrame, const sptam::Map::SharedKeyFrame& targetFrame,
                                            const std::vector<cv::KeyPoint>& kps1, const std::vector<cv::KeyPoint>& kps2,
                                            const std::vector<cv::KeyPoint>& kps3,
                                            const std::vector<SDMatch>& stereo_matches, Matx44d& pose)
{

  points_t point_cloud = triangulateReferencePoints(referenceFrame, kps1, kps2, stereo_matches);

  bearingVectors_t bearingVectors3;

  Eigen::Matrix3d left_calibration = targetFrame->GetFrameLeft().GetCamera().GetIntrinsics();
  double ppal_point_x = left_calibration(0,2);
  double ppal_point_y = left_calibration(1,2);
  double focal_length = left_calibration(0,0);

  /* Bearing vectors of camera 3 (left target camera) towards the trianguled 3d points,
   * we must iterate the points vector in the same way that they were created to ensure corresponding index. */
  for(unsigned int i = 0; i < stereo_matches.size(); i++){
    Point2d kp3 = kps3[stereo_matches[i].m1vs3.trainIdx].pt;

    /* this bearing vector relies on that camera calibration has Z toward the front */
    cv::Vec3d bv( kp3.x - ppal_point_x,
                  kp3.y - ppal_point_y,
                  focal_length);

    bv = cv::normalize(bv);

    bearingVector_t bv3; cv2eigen(bv, bv3);
    bearingVectors3.push_back(bv3);
  }

  // create the central adapter
  absolute_pose::CentralAbsoluteAdapter adapter(bearingVectors3, point_cloud);

  transformation_t estimation;

  // create an AbsolutePoseSacProblem
  // (algorithm is selectable: KNEIP, GAO, or EPNP)
  sac_problems::absolute_pose::AbsolutePoseSacProblem::Algorithm method;

  switch(minimal_method) {
    case MINIMAL_ALGORITHM::KNEIP: method = sac_problems::absolute_pose::AbsolutePoseSacProblem::KNEIP; break;
    case MINIMAL_ALGORITHM::GAO: method = sac_problems::absolute_pose::AbsolutePoseSacProblem::GAO; break;
    case MINIMAL_ALGORITHM::EPNP: method = sac_problems::absolute_pose::AbsolutePoseSacProblem::EPNP; break;
    default: method = sac_problems::absolute_pose::AbsolutePoseSacProblem::KNEIP; break;
  }

  std::shared_ptr<sac_problems::absolute_pose::AbsolutePoseSacProblem>
      method_ptr(new sac_problems::absolute_pose::AbsolutePoseSacProblem(adapter, method));

  // run ransac
  ransac.sac_model_ = method_ptr;
  ransac.computeModel();

  estimation = ransac.model_coefficients_;

  if(ransac.inliers_.size() >= 4){
    /* TODO: evaluate all estimations returned by upnp and select the most adecuate */
    switch(generic_method) {
      case GENERIC_ALGORITHM::NONE: break;
      case GENERIC_ALGORITHM::EPNP: estimation = absolute_pose::epnp(adapter, ransac.inliers_); break;
      case GENERIC_ALGORITHM::UPNP: estimation = absolute_pose::upnp(adapter, ransac.inliers_)[0]; break;
      default: break;
    }

    if(nonlinear_optimization){
      transformation_t nonlinear_transformation;
      method_ptr->optimizeModelCoefficients(ransac.inliers_, estimation, nonlinear_transformation);
      estimation = nonlinear_transformation;
    }
  }

  // openGV returns orientation and position relative to the viewpoint defined (reference frame)
  Eigen::Matrix4d Rt01; Rt01 << estimation(0,0), estimation(0,1), estimation(0,2), estimation(0,3),
                                estimation(1,0), estimation(1,1), estimation(1,2), estimation(1,3),
                                estimation(2,0), estimation(2,1), estimation(2,2), estimation(2,3),
                                0,             0,             0,             1;

  Eigen::Isometry3d Opw0 = Eigen::Isometry3d::Identity();
  Opw0.linear() = referenceFrame->GetCameraPose().GetOrientationMatrix();
  Opw0.translation() = referenceFrame->GetCameraPose().GetPosition();

  Eigen::Isometry3d eig_pose = Opw0 * Eigen::Isometry3d(Rt01);

  /* Rt1w = Rt10 * Rt0w (takes points in worlds coordinate system and returns them in reference of frame 1) */
  pose(0,0) = eig_pose(0,0); pose(0,1) = eig_pose(0,1); pose(0,2) = eig_pose(0,2); pose(0,3) = eig_pose(0,3);
  pose(1,0) = eig_pose(1,0); pose(1,1) = eig_pose(1,1); pose(1,2) = eig_pose(1,2); pose(1,3) = eig_pose(1,3);
  pose(2,0) = eig_pose(2,0); pose(2,1) = eig_pose(2,1); pose(2,2) = eig_pose(2,2); pose(2,3) = eig_pose(2,3);
  pose(3,0) = eig_pose(3,0); pose(3,1) = eig_pose(3,1); pose(3,2) = eig_pose(3,2); pose(3,3) = eig_pose(3,3);

  return ransac.inliers_.size();
}

points_t PoseEstimator::triangulateReferencePoints(const sptam::Map::SharedKeyFrame& referenceFrame,
                                                   const std::vector<cv::KeyPoint>& kps1,
                                                   const std::vector<cv::KeyPoint>& kps2,
                                                   const std::vector<SDMatch>& stereo_matches)
{
  Eigen::Matrix3d left_calibration = referenceFrame->GetFrameLeft().GetCamera().GetIntrinsics();
  double left_ppal_point_x = left_calibration(0,2);
  double left_ppal_point_y = left_calibration(1,2);
  double left_focal_length = left_calibration(0,0);

  Eigen::Matrix3d right_calibration = referenceFrame->GetFrameRight().GetCamera().GetIntrinsics();
  double right_ppal_point_x = right_calibration(0,2);
  double right_ppal_point_y = right_calibration(1,2);
  double right_focal_length = right_calibration(0,0);

  /* Relate keypoints seen from both reference cameras */
  bearingVectors_t bearingVectors1;
  bearingVectors_t bearingVectors2;

  /* Defining corresponding bearing vectors for reference points trangulation using cameras 1 and 2 */
  for(unsigned int i = 0; i < stereo_matches.size(); i++){

    Point2d kp1 = kps1[stereo_matches[i].m1vs2.queryIdx].pt;
    Point2d kp2 = kps2[stereo_matches[i].m1vs2.trainIdx].pt;

    /* this bearing vector relies on that camera calibration has Z toward the front */
    cv::Vec3d cvbv1( kp1.x - left_ppal_point_x,
                     kp1.y - left_ppal_point_y,
                     left_focal_length);
    cvbv1 = cv::normalize(cvbv1);

    cv::Vec3d cvbv2( kp2.x - right_ppal_point_x,
                     kp2.y - right_ppal_point_y,
                     right_focal_length);
    cvbv2 = cv::normalize(cvbv2);

    bearingVector_t bv1; cv2eigen(cvbv1, bv1);
    bearingVector_t bv2; cv2eigen(cvbv2, bv2);

    bearingVectors1.push_back(bv1);
    bearingVectors2.push_back(bv2);
  }

  /* Opengv needs position of right camera in relation with the left camera and rotation from right to left cameras.
   * That is StereoCamera extrinsics parameters in terms of orientation and position of the right camera in reference to
   * the left camera. Usually left camera its centered in the viewpoint of the stereo camera (position 0,0,0 and identity orientation)
   * but StereoCamera framework (and openGV) supports viewpoints not centered on the left cameras. Thats why we've to
   * compute position and orientation of right camera using stereo extrinsics parameters */
  translation_t leftToright_position = referenceFrame->GetFrameLeft().GetCameraPose().GetOrientationMatrix().transpose() * (referenceFrame->GetFrameRight().GetCameraPose().GetPosition() - referenceFrame->GetFrameLeft().GetCameraPose().GetPosition());

  /* left_rotation * right_orientation = T_lw * T_wr = T_lr */
  rotation_t leftToright_orientation = referenceFrame->GetFrameLeft().GetCameraPose().GetOrientationMatrix().transpose() * referenceFrame->GetFrameLeft().GetCameraPose().GetOrientationMatrix();

  relative_pose::CentralRelativeAdapter traigulateAdapter(
      bearingVectors1,
      bearingVectors2,
      leftToright_position,
      leftToright_orientation);

  points_t points;

  for(unsigned int i = 0; i < bearingVectors1.size(); i++)
    points.push_back(triangulation::triangulate(traigulateAdapter, i));
  //point_t point = triangulation::triangulate2( traigulateAdapter, i); // non-linear approach

  return points;
}
