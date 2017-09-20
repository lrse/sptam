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

#include "StereoMatcher.hpp"
#include "../RowMatcher.hpp"

#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include "opencv2/features2d/features2d.hpp"
#include <iostream>
#include <map>
#include <vector>

using namespace std;

StereoMatcher::StereoMatcher(double maxDistance, int normType, bool crossCheck)
  : matcher_( new cv::BFMatcher(normType, crossCheck) ), matchingDistanceThreshold_( maxDistance )
{}

void StereoMatcher::match(const sptam::Map::KeyFrame& stereo_frame1, const sptam::Map::KeyFrame& stereo_frame2,
                          std::vector<SDMatch>& matches) const
{
  // Matching.. use NORM_HAMMING with binary descriptors, NORM_L2 with SIFT and SURF
  RowMatcher matcher(25, cv::NORM_HAMMING);

  vector<cv::DMatch> matches12;

  matcher.match( stereo_frame1.GetFrameLeft().GetFeatures().GetKeypoints(), stereo_frame1.GetFrameLeft().GetFeatures().GetDescriptors(),
                 stereo_frame1.GetFrameRight().GetFeatures().GetKeypoints(), stereo_frame1.GetFrameRight().GetFeatures().GetDescriptors(),
                 matches12);

  vector<cv::DMatch> matches34;

  matcher.match( stereo_frame2.GetFrameLeft().GetFeatures().GetKeypoints(), stereo_frame2.GetFrameLeft().GetFeatures().GetDescriptors(),
                 stereo_frame2.GetFrameRight().GetFeatures().GetKeypoints(), stereo_frame2.GetFrameRight().GetFeatures().GetDescriptors(),
                 matches34);

  StereoMatcher::match( *matcher_, matchingDistanceThreshold_,
                     stereo_frame1.GetFrameLeft().GetFeatures().GetDescriptors(), stereo_frame1.GetFrameRight().GetFeatures().GetDescriptors(),
                     matches12,
                     stereo_frame2.GetFrameLeft().GetFeatures().GetDescriptors(), stereo_frame2.GetFrameRight().GetFeatures().GetDescriptors(),
                     matches34,
                     matches);
}

void StereoMatcher::match(const cv::DescriptorMatcher& matcher, double matchingDistanceThreshold,
                                 const cv::Mat& descriptors1, const cv::Mat& descriptors2, const vector<cv::DMatch>& matches12,
                                 const cv::Mat& descriptors3, const cv::Mat& descriptors4, const vector<cv::DMatch>& matches34,
                                 std::vector<SDMatch>& matches)
{
  std::vector<std::vector<cv::DMatch>> radius_m1vs3;
  matcher.radiusMatch(descriptors1, descriptors3, radius_m1vs3, matchingDistanceThreshold);

  std::vector<std::vector<cv::DMatch>> radius_m2vs4;
  matcher.radiusMatch(descriptors2, descriptors4, radius_m2vs4, matchingDistanceThreshold);

  /* Maps of accepted matches with keys as query indices. This allows us to quickly relate
   * all four frames later. */
  std::map<int,cv::DMatch> map_m1vs3;
  std::map<int,cv::DMatch> map_m2vs4;
  std::map<int,cv::DMatch> map_m3vs4;

  // Lowe's criteria of match selection!
  const float ratio = 0.8;

  /* Checking correspondences consistency */
  for(unsigned int i = 0; i < radius_m1vs3.size(); i++)
    if( ( radius_m1vs3[i].size() == 1 ) || 
        ( radius_m1vs3[i].size() > 1 && radius_m1vs3[i][0].distance < ratio * radius_m1vs3[i][1].distance ) ) // discard too ambiguous correspondences!
      map_m1vs3[(radius_m1vs3[i])[0].queryIdx] = (radius_m1vs3[i])[0];

  /* Checking correspondences consistency */
  for(unsigned int i = 0; i < radius_m2vs4.size(); i++)
    if( ( radius_m2vs4[i].size() == 1 )  ||
        ( radius_m2vs4[i].size() > 1 && radius_m2vs4[i][0].distance < ratio * radius_m2vs4[i][1].distance ) ) // discard too ambiguous correspondences!
      map_m2vs4[(radius_m2vs4[i])[0].queryIdx] = (radius_m2vs4[i])[0];

  for(unsigned int i = 0; i < matches34.size(); i++)
    map_m3vs4[matches34[i].queryIdx] = matches34[i];

  /* Lets build stereo descritor matches, feature relating between frames will be carried
   * in reference of the query stereo pair (that is frames 1 and 2).
   * if a feature its visible by frames 1, 2, 3, 4 in a consistent way, then it will be added. */
  for(unsigned int i = 0; i < matches12.size(); i++){
    auto shared13 = map_m1vs3.find(matches12[i].queryIdx); // Does exist 1<->2<->3 frame relation?
    auto shared24 = map_m2vs4.find(matches12[i].trainIdx); // Does exist 1<->2<->4 frame relation?

    if(shared13 != map_m1vs3.end() && shared24 != map_m2vs4.end()) // there are 1<->2<->3 and 1<->2<->4 relations?
    {
      auto shared34 = map_m3vs4.find(shared13->second.trainIdx); // Does exist a 3<->4 frame relation?

      // there is 3<->4 relation that is consistent with the 1<->2<->4 relation found?
      if(shared34 != map_m3vs4.end() && shared34->second.trainIdx == shared24->second.trainIdx)
        matches.push_back(SDMatch(matches12[i], shared34->second, shared13->second, shared24->second));
    }
  }

  /* TODO: Adding those matches that are visible by 3 of 4 frames */
}

inline void _drawKeypoint(cv::Mat& img, const cv::KeyPoint& p, const cv::Scalar& color)
{
  cv::Point center( cvRound(p.pt.x * 16), cvRound(p.pt.y * 16) );

  int radius = 3 * 16;
  circle( img, center, radius, color, 1, CV_AA, 4);
}

void StereoMatcher::drawStereoMatches(const sptam::Map::SharedKeyFrame& stereo_frame1, const sptam::Map::SharedKeyFrame& stereo_frame2,
                       const std::vector<SDMatch>& stereo_matches, cv::Mat& out)
{

  cv::Mat img1 = cv::Mat::zeros(stereo_frame1->GetFrameLeft().GetCamera().GetCalibration().imageHeight(), stereo_frame1->GetFrameLeft().GetCamera().GetCalibration().imageWidth(), CV_32F);
  cv::Mat img2 = cv::Mat::zeros(stereo_frame1->GetFrameRight().GetCamera().GetCalibration().imageHeight(), stereo_frame1->GetFrameRight().GetCamera().GetCalibration().imageWidth(), CV_32F);
  cv::Mat img3 = cv::Mat::zeros(stereo_frame2->GetFrameLeft().GetCamera().GetCalibration().imageHeight(), stereo_frame2->GetFrameLeft().GetCamera().GetCalibration().imageWidth(), CV_32F);
  cv::Mat img4 = cv::Mat::zeros(stereo_frame2->GetFrameRight().GetCamera().GetCalibration().imageHeight(), stereo_frame2->GetFrameRight().GetCamera().GetCalibration().imageWidth(), CV_32F);

  cv::Size outsize( img1.size().width + img1.size().width, img1.size().height + img1.size().height);
  out.create(outsize, img1.type());
  out = cv::Scalar::all(0);

  // stereo_frame2 at top
  cv::Mat outimg3 = out(cv::Rect(0, 0, img3.size().width, img3.size().height)), outimg4 = out(cv::Rect(img3.size().width, 0, img4.size().width, img4.size().height)),
          outimg1 = out(cv::Rect(0, img3.size().height, img1.size().width, img1.size().height)), outimg2 = out(cv::Rect(img1.size().width, img3.size().height, img2.size().width, img2.size().height));

  img3.copyTo(outimg3);
  img4.copyTo(outimg4);
  img1.copyTo(outimg1);
  img2.copyTo(outimg2);

  // draw matches
  for(unsigned int i = 0; i < stereo_matches.size(); i++){
    SDMatch stereo_match = stereo_matches[i];
    cv::KeyPoint kp1 = stereo_frame1->GetFrameLeft().GetFeatures().GetKeypoints()[stereo_match.m1vs2.queryIdx];
    cv::KeyPoint kp2 = stereo_frame1->GetFrameRight().GetFeatures().GetKeypoints()[stereo_match.m1vs2.trainIdx];
    cv::KeyPoint kp3 = stereo_frame2->GetFrameLeft().GetFeatures().GetKeypoints()[stereo_match.m3vs4.queryIdx];
    cv::KeyPoint kp4 = stereo_frame2->GetFrameRight().GetFeatures().GetKeypoints()[stereo_match.m3vs4.trainIdx];

    cv::RNG& rng = cv::theRNG();
    cv::Scalar color = cv::Scalar(rng(256), rng(256), rng(256));

    _drawKeypoint(outimg1, kp1, color);
    _drawKeypoint(outimg2, kp2, color);
    _drawKeypoint(outimg3, kp3, color);
    _drawKeypoint(outimg4, kp4, color);

    cv::Point2f pt3 = kp3.pt,
            pt4 = cv::Point2f(std::min(kp4.pt.x + outimg3.size().width, float(out.size().width-1)), kp4.pt.y),
            pt1 = cv::Point2f(kp1.pt.x, std::min(kp1.pt.y + outimg3.size().height, float(out.size().height-1))),
            pt2 = cv::Point2f(std::min(kp2.pt.x + outimg1.size().width, float(out.size().width-1)), std::min(kp2.pt.y + outimg4.size().height, float(out.size().height-1)));


    cv::line(out, cv::Point(std::round(pt3.x), std::round(pt3.y)), cv::Point(std::round(pt4.x), std::round(pt4.y)), color, 1, CV_AA);
    cv::line(out, cv::Point(std::round(pt1.x), std::round(pt1.y)), cv::Point(std::round(pt2.x), std::round(pt2.y)), color, 1, CV_AA);
    cv::line(out, cv::Point(std::round(pt1.x), std::round(pt1.y)), cv::Point(std::round(pt3.x), std::round(pt3.y)), color, 1, CV_AA);
    cv::line(out, cv::Point(std::round(pt2.x), std::round(pt2.y)), cv::Point(std::round(pt4.x), std::round(pt4.y)), color, 1, CV_AA);
  }
}
