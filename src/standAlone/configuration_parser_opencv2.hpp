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

  int image_width, image_height;
  config["image_width"] >> image_width;
  config["image_height"] >> image_height;

  double baseline;
  config["baseline"] >> baseline;

  return CameraParameters(cv2eigen<double, 3, 3>(intrinsic), image_width, image_height, frustum_near_plane_distance, frustum_far_plane_distance, baseline);
}

void setParameters( cv::Ptr<cv::Algorithm>&& algorithm, const YAML::Node& node )
{
  std::vector<cv::String> parameters;
  algorithm->getParams( parameters );

  for ( const auto& param : parameters ) {

//    std::cout << param << std::endl;

    if ( node[ param ] )
    {
      int param_type = algorithm->paramType( param );

      switch ( param_type )
      {
        case cv::Param::INT:
        {
          int val = node[ param ].as<int>();
          algorithm->set(param, val);
          std::cout << "  " << param << ": " << val << std::endl;
          #ifdef SHOW_PROFILING
            Logger::Write( "#   " + param + ": " + std::to_string( val ) + "\n" );
          #endif
          break;
        }

        case cv::Param::BOOLEAN:
        {
          bool val = node[ param ].as<bool>();
          algorithm->set(param, val);
          std::cout << "  " << param << ": " << val << std::endl;
          #ifdef SHOW_PROFILING
            Logger::Write( "#   " + param + ": " + std::to_string( val ) + "\n" );
          #endif
          break;
        }

        case cv::Param::REAL:
        {
          double val = node[ param ].as<double>();
          algorithm->set(param, val);
          std::cout << "  " << param << ": " << val << std::endl;
          #ifdef SHOW_PROFILING
            Logger::Write( "#   " + param + ": " + std::to_string( val ) + "\n" );
          #endif
          break;
        }

        case cv::Param::STRING:
        {
          std::string val = node[ param ].as<std::string>();
          algorithm->set(param, val);
          std::cout << "  " << param << ": " << val << std::endl;
          #ifdef SHOW_PROFILING
            Logger::Write( "#   " + param + ": " + val + "\n" );
          #endif
          break;
        }

        //~ case Param::MAT:
        //~ {
          //~ Mat m;
          //~ cv::read(n, m);
          //~ detector->set(param, val);
          //~ break;
        //~ }

        //~ case Param::MAT_VECTOR:
        //~ {
          //~ vector<Mat> mv;
          //~ cv::read(n, mv);
          //~ info->set(algo, pname.c_str(), p.type, &mv, true);
          //~ break;
        //~ }

        //~ case Param::ALGORITHM:
        //~ {
          //~ Ptr<Algorithm> nestedAlgo = Algorithm::_create((string)n["name"]);
          //~ CV_Assert( !nestedAlgo.empty() );
          //~ nestedAlgo->read(n);
          //~ info->set(algo, pname.c_str(), p.type, &nestedAlgo, true);
          //~ break;
        //~ }

        default:
          //~ CV_Error( CV_StsUnsupportedFormat, "unknown/unsupported parameter type");
          std::cerr << "unknown/unsupported parameter type for " << param << std::endl;
          break;
      }
    }
  }
}

cv::Ptr<cv::FeatureDetector> loadFeatureDetector( const YAML::Node& node )
{
  const std::string name = node["Name"].as<std::string>();
  std::cout << "detector: " << name << std::endl;

  #ifdef SHOW_PROFILING
    Logger::Write( "#   detector: " + name + "\n" );
  #endif

  cv::Ptr<cv::FeatureDetector> detector = cv::FeatureDetector::create( name );

  if ( not detector ) {
    std::cout << "could not load detector with name " << name << std::endl;
    return nullptr;
  }

  setParameters( detector, node );

  return detector;
}

cv::Ptr<cv::DescriptorExtractor> loadDescriptorExtractor( const YAML::Node& node )
{
  const std::string name = node["Name"].as<std::string>();
  std::cout << "descriptor: " << name << std::endl;

  #ifdef SHOW_PROFILING
    Logger::Write( "#   descriptor: " + name + "\n" );
  #endif


  cv::Ptr<cv::DescriptorExtractor> extractor = cv::DescriptorExtractor::create( name );

  if ( not extractor ) {
    std::cout << "could not load extractor with name " << name << std::endl;
    return nullptr;
  }

  setParameters( extractor, node );

  return extractor;
}

cv::Ptr<cv::DescriptorMatcher> loadDescriptorMatcher( const YAML::Node& node )
{
  const std::string name = node["Name"].as<std::string>();
  std::cout << "matcher: " << name << std::endl;

  #ifdef SHOW_PROFILING
    Logger::Write( "#   descriptor: " + name + "\n" );
  #endif

  cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create( name );

  if ( not matcher ) {
    std::cout << "could not load matcher with name " << name << std::endl;
    return nullptr;
  }

  setParameters( matcher, node );

  return matcher;
}
