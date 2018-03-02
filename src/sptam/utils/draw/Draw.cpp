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

#include "Draw.hpp"
#include "../macros.hpp"
#include "../../MEAS.hpp"
#include "../projective_math.hpp"
#include "../covariance_ellipsoid.hpp"
#include "../eigen_alignment.hpp"

// Confidence interval thresholds for 2 parameter function
#define CHI2_THRESH_95 5.991
#define CHI2_THRESH_99 9.210

void drawLine(cv::Mat& img, const cv::Point2d& p1, const cv::Point2d& p2, cv::Scalar color)
{
  cv::line(img, cv::Point(p1.x, p1.y), cv::Point(p2.x, p2.y), color);
}

/**
 * Draw a point as a cross on the image
 */
void drawPoint(cv::Mat& img, const cv::Point2d& point, cv::Scalar color)
{
  cv::line(img, cv::Point(point.x, point.y - 4), cv::Point(point.x, point.y + 4), color);
  cv::line(img, cv::Point(point.x + 4, point.y), cv::Point(point.x - 4, point.y), color);
}

/**
 * Draw a set of features and their projections onto an image
 */
void drawFeatures(const sptam::Map::KeyFrame& frame, cv::Mat& image)
{
  // Get Projection Matrix
  Eigen::Matrix34d projectionLeft = frame.GetFrameLeft().GetProjection();

  // Draw matches
  for( const auto& measurement : frame.measurements() )
  {
    if (measurement->GetType() != Measurement::RIGHT)
    {
      const MapPoint& mapPoint = *measurement->mapPoint();

      cv::Point2d projectedPoint = project(projectionLeft, mapPoint.GetPosition());
      const cv::KeyPoint& featurePos_v = measurement->GetKeypoints()[0];
      cv::Point2d featurePos(featurePos_v.pt.x, featurePos_v.pt.y);

      // Dibujo la linea desde el feature detectado al feature predicho
      drawLine(image, featurePos, projectedPoint, COLOR_GREEN);

      drawPoint(image, projectedPoint, COLOR_YELLOW);

      drawPoint(image, featurePos, COLOR_RED);
    }

  }

  // Draw unmatched keyPoints
//  std::vector<cv::KeyPoint> unMatchedkeyPoints;
//  cv::Mat descriptors;
//  std::vector<size_t> indexes;
//  frame.GetUnmatchedKeyPoints(unMatchedkeyPoints,descriptors,indexes);
//  cv::drawKeypoints(image, unMatchedkeyPoints, image, COLOR_BLUE);

}

/**
 * Draw a set of features an their projections onto an image
 */
void drawFeatures(const Eigen::Matrix34d& projection, const std::aligned_vector<Eigen::Vector3d>& points, const std::vector<MEAS>& features,
                  const cv::Scalar& color_proj, const cv::Scalar& color_feat, cv::Mat& image)
{
  // Draw matches
  forn( i, points.size() ) {

    const cv::Point2d& featurePos = features[ i ].keypoint.pt;
    cv::Point2d projectedPoint = project(projection, points[ i ]);

    // Dibujo la linea desde el feature detectado al feature predicho
    drawLine(image, featurePos, projectedPoint, COLOR_GREEN);

    drawPoint(image, projectedPoint, color_proj);
    drawPoint(image, featurePos, color_feat);
  }
}

#include "../projection_derivatives.hpp"

/**
 * Draw the projections of a set of points onto an image recorded by a camera,
 * as well as the corresponding covariance ellipses.
 */
template<class T>
void drawProjectionCovariances(const Frame& frame, cv::Mat& image, const T& points)
{
  const Eigen::Matrix<double,3,4>& transformation = frame.GetCamera().GetTransformation();

  const Eigen::Matrix6d& pose_covariance = frame.GetCameraPose().covariance();

  Eigen::Matrix9d parameter_covariance = Eigen::Matrix9d::Zero();
  parameter_covariance.block<6, 6>(0, 0) = pose_covariance;

  Eigen::Matrix3d K = frame.GetCamera().GetIntrinsics();

  for ( const sptam::Map::SharedPoint& mapPoint : points )
  {
    parameter_covariance.block<3, 3>(6, 6) = mapPoint->covariance();

    cv::Point2d projectedPoint = project(frame.GetProjection(), mapPoint->GetPosition());

    ////////////////////////////////////////////////////////////////////////////
    // compute projection covariance
    ////////////////////////////////////////////////////////////////////////////

    Eigen::Matrix<double,2,6> jXj = jacobianXj(transformation, K, mapPoint->GetPosition());
    Eigen::Matrix<double,2,3> jXi = jacobianXi(transformation, K, mapPoint->GetPosition());

    // propagate both point and camera covariances
    Eigen::Matrix<double, 2, 9> J;
    J.block<2, 6>(0, 0) = jXj;
    J.block<2, 3>(0, 6) = jXi;
    // Eigen::Matrix2d projection_covariance = J * parameter_covariance * J.transpose();

    // asume perfect 3D points and just propagate pose covariance
    Eigen::Matrix2d projection_covariance = jXj * pose_covariance * jXj.transpose();

    // asume perfect camera pose and just propagate point covariances
    //~ Eigen::Matrix2d projection_covariance = jXi * point_covariance * jXi.transpose();

    ////////////////////////////////////////////////////////////////////////////
    // Compute 2D covariance ellipse
    ////////////////////////////////////////////////////////////////////////////

    // 99% confidence ellipse
    Ellipse2D ellipse = computeCovarianceEllipse(projection_covariance, PROB_95);

    // To obtain the orientation of the ellipse, we simply calculate
    // the angle of the largest eigenvector towards the x-axis.
    double angle = atan2( ellipse.ax1[1], ellipse.ax1[0] );

    cv::ellipse(image, projectedPoint, cv::Size(ellipse.len1, ellipse.len2), angle, 0, 360, COLOR_GREEN);
  }
}

template void drawProjectionCovariances<sptam::Map::SharedMapPointList>(const Frame& frame, cv::Mat& image, const sptam::Map::SharedMapPointList& points);
template void drawProjectionCovariances<sptam::Map::SharedMapPointSet>(const Frame& frame, cv::Mat& image, const sptam::Map::SharedMapPointSet& points);

void drawFrame(const sptam::Map::KeyFrame& frame, const cv::Mat& image, cv::Mat& outImage)
{
  outImage = image.clone();
  drawFeatures(frame, outImage);
}

cv::Mat drawFrame(const sptam::Map::KeyFrame& frame, const cv::Mat& image)
{
  cv::Mat outImage;
  drawFrame(frame, image, outImage);

  return outImage;
}

cv::Mat drawFrame(const sptam::Map::KeyFrame& frame)
{
  // TODO: this is hardcoding for KITTI dataset
  // now these values are the size of the KITTI images.
  cv::Mat image = cv::Mat::zeros(376, 1241, CV_8UC3);
  cv::Mat outImage;
  drawFrame(frame, image, outImage);

  return outImage;
}

/**
 * make a new image merging two stereo frames and return references to each sub-frame.
 */
cv::Mat makeStereoWindow(const cv::Mat& imgL, const cv::Mat& imgR, cv::Mat& outImageLeft, cv::Mat& outImageRight, bool horizontal)
{
  size_t width =  horizontal ? imgL.cols + imgR.cols : std::max(imgL.cols, imgR.cols) ;
  size_t height = horizontal ? std::max(imgL.rows, imgR.rows) : imgL.rows + imgR.rows ;

  cv::Mat outImage(height, width, CV_8UC3);

  outImageLeft = cv::Mat(outImage, cv::Rect(0, 0, imgL.cols, imgL.rows));

  size_t offset_x = horizontal ? imgL.cols : 0 ;
  size_t offset_y = horizontal ? 0 : imgL.rows ;

  outImageRight = cv::Mat(outImage, cv::Rect(offset_x, offset_y, imgR.cols, imgR.rows));

  return outImage;
}

void makeColorCopy(const cv::Mat& in, cv::Mat& out)
{
  if (in.type() == CV_8UC1)
    cvtColor(in, out, CV_GRAY2BGR);
  else
    in.copyTo(out);
}

void drawGrid(cv::Mat& image, const size_t cell_size, const cv::Scalar& color)
{
  // check that cell_size is possitive
  if ( not (0 < cell_size) ) {
    std::cerr << "Grid: cell size is not possitive" << std::endl;
    return;
  }

  int width = image.size().width;
  int height = image.size().height;

  for(int i=0; i<height; i+=cell_size)
    cv::line(image, cv::Point2d(0,i), cv::Point2d(width,i), color);

  for(int i=0; i<width; i+=cell_size)
    cv::line(image, cv::Point2d(i,0), cv::Point2d(i,height), color);
}

void drawProjections(cv::Mat& image, const Eigen::Matrix34d& projection, const ConstIterable<sptam::Map::Point>& mapPoints, const cv::Scalar& color)
{
  for (const sptam::Map::Point& mapPoint : mapPoints)
  {
    cv::Point2d projecion = project(projection, mapPoint.GetPosition());
    drawPoint(image, projecion, color);
  }
}

void drawMeasurements(const Eigen::Matrix34d& projection, const std::list<Match>& measurements,
                  const cv::Scalar& color_proj, const cv::Scalar& color_feat, cv::Mat& image)
{
  // Draw matches
  for( const Match& match : measurements  )
  {
    const cv::KeyPoint& featurePos_v = match.measurement.GetKeypoints()[0];
    cv::Point2d featurePos(featurePos_v.pt.x, featurePos_v.pt.y);
    cv::Point2d projectedPoint = project(projection, match.mapPoint->GetPosition());

    // Dibujo la linea desde el feature detectado al feature predicho
    drawLine(image, featurePos, projectedPoint, COLOR_GREEN);

    drawPoint(image, projectedPoint, color_proj);
    drawPoint(image, featurePos, color_feat);
  }
}

void drawMeasuredFeatures(const StereoFrame& frame, const std::list<Match>& measurements, cv::Mat& imgL, cv::Mat& imgR)
{
  // get left and right measurements. Stereo are share.
  std::list<Match> meas_left, meas_right;
  for ( const auto& match : measurements )
  {
    if (match.measurement.GetType() == Measurement::LEFT)
      meas_left.push_back( match );

    if (match.measurement.GetType() == Measurement::RIGHT)
      meas_right.push_back( match );

    if (match.measurement.GetType() == Measurement::STEREO)
    {
      meas_left.push_back( {match.mapPoint, Measurement(Measurement::LEFT, match.measurement.GetSource(), match.measurement.GetKeypoints()[0], match.mapPoint->GetDescriptor())} );
      meas_right.push_back( {match.mapPoint, Measurement(Measurement::RIGHT, match.measurement.GetSource(), match.measurement.GetKeypoints()[1], match.mapPoint->GetDescriptor())} );
    }
  }

  // get projections matrices
  const Eigen::Matrix34d projection_left = frame.GetFrameLeft().GetProjection();
  const Eigen::Matrix34d projection_right = frame.GetFrameRight().GetProjection();

  // draw measurements
  drawMeasurements(projection_left, meas_left, COLOR_YELLOW, COLOR_RED, imgL);
  drawMeasurements(projection_right, meas_right, COLOR_YELLOW, COLOR_RED, imgR);
}

void drawMeasuredFeatures(const StereoFrame& frame, const std::list<Match>& measurements, cv::Mat& img, bool left)
{
  std::list<Match> meas;
  for ( const auto& match : measurements )
  {
    if (match.measurement.GetType() == Measurement::LEFT) { if (left) meas.push_back( match ); }
    else if (match.measurement.GetType() == Measurement::RIGHT) { if (!left) meas.push_back( match ); }
    else if (match.measurement.GetType() == Measurement::STEREO)
    {
      if (left)
        meas.push_back( {match.mapPoint, Measurement(Measurement::LEFT, match.measurement.GetSource(), match.measurement.GetKeypoints()[0], match.mapPoint->GetDescriptor())} );
      else
        meas.push_back( {match.mapPoint, Measurement(Measurement::RIGHT, match.measurement.GetSource(), match.measurement.GetKeypoints()[1], match.mapPoint->GetDescriptor())} );
    }
  }

  const Eigen::Matrix34d projection = (left ? frame.GetFrameLeft() : frame.GetFrameRight()).GetProjection();
  drawMeasurements(projection, meas, COLOR_YELLOW, COLOR_RED, img);
}

void saveFrame(const sptam::Map::KeyFrame& frame, const cv::Mat& image, const std::string& fileName)
{
  cv::Mat outImage = drawFrame( frame, image );

  // save image
  std::stringstream sstm;
  sstm << fileName << ".jpg";

  cv::imwrite(sstm.str(), outImage);
}

// TODO From here to the end of the file, we need to check what is useful and what not

void DrawEpipolarLine(cv::Mat& image,
                      const std::vector<cv::Point2d>& inlierPointsLeft,
                      const std::vector<cv::Point2d>& inlierPointsRight,
                      const cv::Matx33d& fundamentalMatrix,
                      std::vector<cv::Vec3f>& epipolarLinesRight)
{
  cv::computeCorrespondEpilines(inlierPointsLeft, 1, fundamentalMatrix, epipolarLinesRight);

  for (unsigned int i = 0; i < epipolarLinesRight.size(); ++i) {
    DrawOneEpipolarLine(image, epipolarLinesRight[i]);
    cv::Point2d point = inlierPointsRight[i]; //casting to pass inlierPointRight to drawPoint()
    drawPoint(image, point, cv::Scalar(0, 0, 255));
  }
}

// DrawOneEpipolarLine dibuja una linea en una imagen, esta linea cruza completamente la imagen
void DrawOneEpipolarLine(cv::Mat& image, cv::Vec3f& line)
{
  // draw the line between first and last column
  cv::line(image,cv::Point(0,-line[2]/line[1]), cv::Point(image.cols,-(line[2]+ line[0]*image.cols)/line[1]),cv::Scalar(255,0,0));
  return;
}

// This function is similar to OpenCV drawMatches function with the difference it show one image over the other.
void DrawMatches(const cv::Mat& img1, const std::vector<cv::KeyPoint>& keypoints1,
                 const cv::Mat& img2, const std::vector<cv::KeyPoint>& keypoints2,
                 const std::vector<cv::DMatch>& matches, cv::Mat& outImage)
{
    unsigned int width = img1.cols;
    unsigned int height = img1.rows;
    outImage.create(height * 2, width, img1.type());
    img1.copyTo(outImage(cv::Rect(0, 0, width, height)));
    img2.copyTo(outImage(cv::Rect(0, height, width, height)));

    for (uint i = 0; i < matches.size(); ++i) {

      cv::DMatch match = matches[i];
      cv::KeyPoint keyPoint1 = keypoints1[match.queryIdx];
      cv::KeyPoint keyPoint2 = keypoints2[match.trainIdx];


      const cv::Point2d& point1 = keyPoint1.pt;
      const cv::Point2d& point2 = cv::Point2d(keyPoint2.pt.x, keyPoint2.pt.y + height);

      drawPoint(outImage, point1, COLOR_RED);

      drawPoint(outImage, point2, COLOR_RED);

      drawLine(outImage, point1, point2, COLOR_GREEN);
    }
}
