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

#include "Draw.hpp"
#include "macros.hpp"
#include "projective_math.hpp"

#include <opencv2/highgui/highgui.hpp>

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
void drawFeatures(const Frame& frame, cv::Mat& image)
{
  // Get Projection Matrix
  cv::Matx34d projectionLeft = frame.GetProjection();

  // Draw matches
  for( auto& kv : frame.GetMeasurements() ) {

    MapPoint* mapPoint = kv.first;
    const Measurement& measurement = kv.second;

    cv::Point2d projectedPoint = project( projectionLeft, mapPoint->GetPosition() );
    cv::Point2d featurePos = measurement.GetProjection();

    // Dibujo la linea desde el feature detectado al feature predicho
    drawLine(image, featurePos, projectedPoint, COLOR_GREEN);

    drawPoint(image, projectedPoint, COLOR_YELLOW);

    drawPoint(image, featurePos, COLOR_RED);

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
void drawFeatures(const cv::Matx34d& projection, const std::vector<cv::Point3d>& points, const std::vector<MEAS>& features,
                  const cv::Scalar& color_proj, const cv::Scalar& color_feat, cv::Mat& image)
{
  // Draw matches
  forn( i, points.size() ) {

    const cv::Point2d& featurePos = features[ i ].projection;
    cv::Point2d projectedPoint = project( projection, points[ i ] );

    // Dibujo la linea desde el feature detectado al feature predicho
    drawLine(image, featurePos, projectedPoint, COLOR_GREEN);

    drawPoint(image, projectedPoint, color_proj);
    drawPoint(image, featurePos, color_feat);
  }
}

void drawFeatures(const cv::Matx34d& projection, const std::vector<Measurement>& measurements,
                  const cv::Scalar& color_proj, const cv::Scalar& color_feat, cv::Mat& image)
{
  // Draw matches
  for( const auto& meas : measurements  ) {

    const cv::Point2d& featurePos = meas.GetProjection();
    cv::Point2d projectedPoint = project( projection, meas.mapPoint->GetPosition() );

    // Dibujo la linea desde el feature detectado al feature predicho
    drawLine(image, featurePos, projectedPoint, COLOR_GREEN);

    drawPoint(image, projectedPoint, color_proj);
    drawPoint(image, featurePos, color_feat);
  }
}

void drawFrame(const Frame& frame, const cv::Mat& image, cv::Mat& outImage)
{
	outImage = image.clone();
  drawFeatures(frame, outImage);
}

cv::Mat drawFrame(const Frame& frame, const cv::Mat& image)
{
	cv::Mat outImage;
  drawFrame(frame, image, outImage);

	return outImage;
}

cv::Mat drawFrame(const Frame& frame)
{
  // TODO: esto esta hardcodeado para debugging
  // es el tamaño de las imagenes de KITTI
  cv::Mat image = cv::Mat::zeros(376, 1241, CV_8UC3);
  cv::Mat outImage;
  drawFrame(frame, image, outImage);

  return outImage;
}


cv::Mat drawTrackedFrames(const StereoFrame& frame, const cv::Mat& imgL, const cv::Mat& imgR) {

  cv::Mat outImageLeft, outImageRight;
  return drawTrackedFrames(frame, imgL, imgR, outImageLeft, outImageRight);
}


cv::Mat drawTrackedFrames(const StereoFrame& frame, const cv::Mat& imgL, const cv::Mat& imgR, cv::Mat& outImageLeft, cv::Mat& outImageRight)
{
  cv::Mat outImage(imgL.rows + imgR.rows, std::max(imgL.cols, imgR.cols), imgL.type());

  outImageLeft = cv::Mat(outImage, cv::Rect(0, 0, imgL.cols, imgL.rows));
  imgL.copyTo( outImageLeft );
  drawFeatures(frame.GetFrameLeft(), outImageLeft);

  outImageRight = cv::Mat(outImage, cv::Rect(0, imgL.rows, imgR.cols, imgR.rows));
  imgR.copyTo( outImageRight );
  drawFeatures(frame.GetFrameRight(), outImageRight);

  return outImage;
}

void saveFrame(const Frame& frame, const cv::Mat& image, const std::string& fileName)
{
  cv::Mat outImage = drawFrame( frame, image );

  // save image
  std::stringstream sstm;
  sstm << fileName << ".jpg";

  cv::imwrite(sstm.str(), outImage);
}

// TODO de acá para abajo falta revisar que sirve / que no y encapsular varias cosas

/**
 * Save Stereo KeyFrame whit its matches
 */
void saveStereoKeyFrame(const StereoFrame& keyFrame, const cv::Mat& _imageLeft, const cv::Mat& _imageRight, const std::string fileName)
{

  cv::Mat imageLeft = drawFrame( keyFrame.GetFrameLeft(), _imageLeft );
  cv::Mat imageRight = drawFrame( keyFrame.GetFrameRight(), _imageRight );

  unsigned int width = imageLeft.cols;
  unsigned int height = imageRight.rows;
  cv::Mat outImage = cv::Mat(height * 2, width, imageLeft.type());
  imageLeft.copyTo(outImage(cv::Rect(0, 0, width, height)));
  imageRight.copyTo(outImage(cv::Rect(0, height, width, height)));

  // save image
  std::stringstream sstm;
  sstm << "./" << fileName << "_KeyFrame_" << keyFrame.GetId() << ".jpg";
  cv::imwrite(sstm.str(), outImage);
}





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
