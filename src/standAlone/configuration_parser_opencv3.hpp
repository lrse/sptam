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

#include "../sptam/utils/projective_math.hpp"
#include "../sptam/utils/cv2eigen.hpp"
#include "../sptam/CameraParameters.hpp"

#ifdef SHOW_PROFILING
  #include "../sptam/utils/log/Logger.hpp"
#endif

#include <iostream>
#include <yaml-cpp/yaml.h>
#include "opencv2/core/version.hpp"
#if CV_MAJOR_VERSION == 2
  #include <opencv2/core/core.hpp>
  #include <opencv2/nonfree/nonfree.hpp>
#elif CV_MAJOR_VERSION == 3
  #include <opencv2/core.hpp>
  #include <opencv2/xfeatures2d.hpp>
#endif

CameraParameters loadCameraCalibration(const std::string& filename, double frustum_near_plane_distance, double frustum_far_plane_distance)
{
  cv::FileStorage config(filename, cv::FileStorage::READ);

  if (not config.isOpened())
    throw std::invalid_argument("Error opening rectified camera calibration file " + filename);

  cv::Mat_<double> intrinsic;
  config["camera_matrix"] >> intrinsic;

  return CameraParameters(cv2eigen<double, 3, 3>(intrinsic), (int) config["image_width"], (int) config["image_height"], frustum_near_plane_distance, frustum_far_plane_distance, (double) config["baseline"]);
}

template<typename T>
T readParameter(const YAML::Node& node, const std::string &parameterName, const T& defaultValue)
{
  T value;
  if ( node[parameterName].IsDefined() ) {
    value = node[parameterName].as<T>();
  } else {
    value = defaultValue;
  }
  std::cout << "  " << parameterName << ": " << value << std::endl;

  #ifdef SHOW_PROFILING
    Logger::Write( "#   " + parameterName + ": " + std::to_string( value ) + "\n" );
  #endif

  return value;
}

cv::Ptr<cv::Feature2D>loadORB(const YAML::Node& node)
{
  // load parameters
  int nFeatures;
  nFeatures = readParameter<int>(node, "nFeatures", 500);

  float scaleFactor;
  scaleFactor = readParameter<float>(node, "scaleFactor", 1.2f);

  int nLevels;
  nLevels = readParameter<int>(node, "nLevels", 8);

  int edgeThreshold;
  edgeThreshold = readParameter<int>(node, "edgeThreshold", 31);

  int firstLevel;
  firstLevel = readParameter<int>(node, "firstLevel", 0);

  int WTA_K;
  WTA_K = readParameter<int>(node, "WTA_K", 2);

  int scoreType;
  scoreType = readParameter<int>(node, "scoreType", cv::ORB::HARRIS_SCORE);

  int patchSize;
  patchSize = readParameter<int>(node, "patchSize", 31);

  int fastThreshold;
  fastThreshold = readParameter<int>(node, "fastThreshold", 20);

  return cv::ORB::create(nFeatures, scaleFactor, nLevels, edgeThreshold, firstLevel, WTA_K, scoreType, patchSize, fastThreshold);
}

cv::Ptr<cv::Feature2D>loadGFTT(const YAML::Node& node)
{
  int nfeatures;
  nfeatures = readParameter<int>(node, "nfeatures", 1000);


  double qualityLevel;
  qualityLevel = readParameter<double>(node, "qualityLevel", 0.01);

  double minDistance;
  minDistance = readParameter<double>(node, "minDistance", 1);


  int blockSize;
  blockSize = readParameter<int>(node, "blockSize", 3);

  bool useHarrisDetector;
  useHarrisDetector = readParameter<bool>(node, "useHarrisDetector", false);

  double k;
  k = readParameter<double>(node, "k", 0.04);

  // create detector
  return cv::GFTTDetector::create(nfeatures, qualityLevel, minDistance, blockSize, useHarrisDetector, k);
}

cv::Ptr<cv::Feature2D>loadSTAR(const YAML::Node& node)
{
  // load parameters
  int maxSize;
  maxSize = readParameter<int>(node, "maxSize", 45);

  int responseThreshold;
  responseThreshold = readParameter<int>(node, "responseThreshold", 30);

  int lineThresholdProjected;
  lineThresholdProjected = readParameter<int>(node, "lineThresholdProjected", 10);

  int lineThresholdBinarized;
  lineThresholdBinarized = readParameter<int>(node, "lineThresholdBinarized", 8);

  int suppressNonmaxSize;
  suppressNonmaxSize = readParameter<int>(node, "suppressNonmaxSize", 5);

  return cv::xfeatures2d::StarDetector::create(maxSize, responseThreshold, lineThresholdProjected, lineThresholdBinarized, suppressNonmaxSize);
}

cv::Ptr<cv::Feature2D>loadSURF(const YAML::Node& node)
{
  // load parameters
  float hessianThreshold;
  hessianThreshold = readParameter<float>(node, "hessianThreshold", 100.0f);

  int nOctaves;
  nOctaves = readParameter<int>(node, "nOctaves", 4);

  int nOctaveLayers;
  nOctaveLayers = readParameter<int>(node, "nOctaveLayers", 3);

  bool extended;
  extended = readParameter<bool>(node, "extended", false);

  bool upright;
  upright = readParameter<bool>(node, "upright", false);

  return cv::xfeatures2d::SURF::create(hessianThreshold, nOctaves, nOctaveLayers, extended, upright);
}

cv::Ptr<cv::Feature2D>loadFAST(const YAML::Node& node)
{
  // load parameters
  int threshold;
  threshold = readParameter<int>(node, "threshold", 10);

  bool nonmaxSuppression;
  nonmaxSuppression = readParameter<bool>(node, "nonmaxSuppression", true);

  int type;
  type = readParameter<int>(node, "type", cv::FastFeatureDetector::TYPE_9_16);

  return cv::FastFeatureDetector::create(threshold, nonmaxSuppression, type);
}

cv::Ptr<cv::Feature2D>loadAGAST(const YAML::Node& node)
{
  // load parameters
  int threshold;
  threshold = readParameter<int>(node, "threshold", 10);

  bool nonmaxSuppression;
  nonmaxSuppression = readParameter<bool>(node, "nonmaxSuppression", true);

  int type;
  type = readParameter<int>(node, "type", cv::AgastFeatureDetector::OAST_9_16);

  return cv::AgastFeatureDetector::create(threshold, nonmaxSuppression, type);
}

cv::Ptr<cv::Feature2D>loadAKAZE(const YAML::Node& node)
{
  // load parameters
  int descriptor_type;
  descriptor_type = readParameter<int>(node, "descriptor_type", cv::AKAZE::DESCRIPTOR_MLDB);

  int descriptor_size;
  descriptor_size = readParameter<int>(node, "descriptor_size", 0);

  int descriptor_channels;
  descriptor_channels = readParameter<int>(node, "descriptor_channels", 3);

  float threshold;
  threshold = readParameter<float>(node, "threshold", 0.001f);

  int nOctaves;
  nOctaves = readParameter<int>(node, "nOctaves", 4);

  int nOctaveLayers;
  nOctaveLayers = readParameter<int>(node, "nOctaveLayers", 4);

  int diffusivity;
  diffusivity = readParameter<int>(node, "diffusivity", cv::KAZE::DIFF_PM_G2);

  return cv::AKAZE::create(descriptor_type, descriptor_size, descriptor_channels, threshold, nOctaves, nOctaveLayers, diffusivity);
}

cv::Ptr<cv::Feature2D>loadBRIEF(const YAML::Node& node)
{
  // load parameters
  int bytes;
  bytes = readParameter<int>(node, "bytes", 32);

  bool use_orientation;
  use_orientation = readParameter<bool>(node, "use_orientation", false);

  return cv::xfeatures2d::BriefDescriptorExtractor::create( bytes, use_orientation );
}

cv::Ptr<cv::Feature2D>loadBRISK(const YAML::Node& node)
{
  // load parameters
  int thresh;
  thresh = readParameter<int>(node, "thresh", 30);

  int octaves;
  octaves = readParameter<int>(node, "octaves", 3);

  float patternScale;
  patternScale = readParameter<float>(node, "patternScale", 1.0f);

  return cv::BRISK::create(thresh, octaves, patternScale);
}

cv::Ptr<cv::Feature2D>loadLATCH(const YAML::Node& node)
{
  // load parameters
  int bytes;
  bytes = readParameter<int>(node, "bytes", 32);

  bool rotationInvariance;
  rotationInvariance = readParameter<bool>(node, "rotationInvariance", true);

  int half_ssd_size;
  half_ssd_size = readParameter<int>(node, "half_ssd_size", 3);

  return cv::xfeatures2d::LATCH::create(bytes, rotationInvariance, half_ssd_size);
}

cv::Ptr<cv::Feature2D>loadLUCID(const YAML::Node& node)
{
  // load parameters
  int lucid_kernel;
  lucid_kernel = readParameter<int>(node, "lucid_kernel", 1);

  int blur_kernel;
  blur_kernel = readParameter<int>(node, "blur_kernel", 1);

  return cv::xfeatures2d::LUCID::create(lucid_kernel, blur_kernel);
}

cv::Ptr<cv::Feature2D>loadFREAK(const YAML::Node& node)
{
  // load parameters
  bool orientationNormalized;
  orientationNormalized = readParameter<bool>(node, "orientationNormalized", true);

  bool scaleNormalized;
  scaleNormalized = readParameter<bool>(node, "scaleNormalized", true);

  float patternScale;
  patternScale = readParameter<float>(node, "patternScale", 22.0f);

  int nOctaves;
  nOctaves = readParameter<int>(node, "nOctaves", 4);

  const std::vector<int>& selectedPairs = std::vector<int>();

  return cv::xfeatures2d::FREAK::create(orientationNormalized, scaleNormalized, patternScale, nOctaves, selectedPairs);
}

cv::Ptr<cv::FeatureDetector> loadFeatureDetector( const YAML::Node& node )
{
  const std::string name = node["Name"].as<std::string>();
  std::cout << "detector: " << name << std::endl;

  #ifdef SHOW_PROFILING
    Logger::Write( "#   detector: " + name + "\n" );
  #endif

  cv::Ptr<cv::FeatureDetector> detector;
  if ( name.compare("GFTT") == 0 ) {
    detector = loadGFTT( node );
  }
  else if (name.compare("ORB") == 0) {
    detector = loadORB( node );
  }
  else if (name.compare("STAR") == 0) {
    detector = loadSTAR( node );
  }
  else if (name.compare("BRISK") == 0) {
    detector = loadBRISK( node );
  }
  else if (name.compare("FAST") == 0) {
    detector = loadFAST( node );
  }
  else if (name.compare("AGAST") == 0) {
    detector = loadAGAST( node );
  }
  else if (name.compare("AKAZE") == 0) {
    detector = loadAKAZE( node );
  }
  else if (name.compare("SURF") == 0) {
    detector = loadSURF( node );
  }

  if ( not detector ) {
    std::cerr << "could not load detector with name " << name << std::endl;
  }

  return detector;
}

cv::Ptr<cv::DescriptorExtractor> loadDescriptorExtractor( const YAML::Node& node )
{
  const std::string name = node["Name"].as<std::string>();
  std::cout << "descriptor: " << name << std::endl;

  #ifdef SHOW_PROFILING
    Logger::Write( "#   descriptor: " + name + "\n" );
  #endif

  cv::Ptr<cv::DescriptorExtractor> extractor;
  if ( name.compare("BRIEF") == 0 ) {
    extractor = loadBRIEF( node );
  }
  else if (name.compare("BRISK") == 0) {
    extractor = loadBRISK( node );
  }
  else if (name.compare("ORB") == 0) {
    extractor = loadORB( node );
  }
  else if (name.compare("FREAK") == 0) {
    extractor = loadFREAK( node );
  }
  else if (name.compare("LATCH") == 0) {
    extractor = loadLATCH( node );
  }
  else if (name.compare("LUCID") == 0) {
    extractor = loadLUCID( node );
  }
  else if (name.compare("AKAZE") == 0) {
    extractor = loadAKAZE( node );
  }
  else if (name.compare("SURF") == 0) {
    extractor = loadSURF( node );
  }

  if ( not extractor ) {
    std::cerr << "could not load extractor with name " << name << std::endl;
  }

  return extractor;
}

cv::Ptr<cv::DescriptorMatcher> loadDescriptorMatcher( const YAML::Node& node )
{
  const std::string name = node["Name"].as<std::string>();
  std::cout << "matcher: " << name << std::endl;

  cv::Ptr<cv::DescriptorMatcher> matcher;

  matcher = cv::DescriptorMatcher::create( name );

  if ( not matcher ) {
    std::cerr << "could not load matcher with name " << name << std::endl;
  }

  return matcher;
}
