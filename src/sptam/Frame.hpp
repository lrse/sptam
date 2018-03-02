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

#include "Camera.hpp"
#include "Measurement.hpp"
#include "ImageFeatures.hpp"
#include "utils/eigen_alignment.hpp"

#include "opencv2/core/version.hpp"
#if CV_MAJOR_VERSION == 2
  #include <opencv2/core/core.hpp>
#elif CV_MAJOR_VERSION == 3
  #include <opencv2/core.hpp>
#endif

class Frame
{
  public:

    Frame(const Camera& camera, const ImageFeatures& imageFeatures);

    inline const Eigen::Vector3d& GetPosition() const
    { return camera_.GetPosition(); }

    inline const Eigen::Quaterniond& GetOrientation() const
    { return camera_.GetOrientation(); }

    inline const Camera& GetCamera() const
    { return camera_; }

    inline const CameraPose& GetCameraPose() const
    { return camera_.GetPose(); }

    inline Eigen::Matrix34d GetProjection() const
    { return camera_.GetProjection(); }

    inline void UpdateCameraPose(const CameraPose& cameraPose)
    { camera_.UpdatePose( cameraPose ); }

    inline const ImageFeatures& GetFeatures() const{
      return imageFeatures_;
    }

    /**
     * Match a set of 3D points with their respective descriptors
     * to the features in the current frame.
     */
    std::list<std::pair<size_t, size_t> > FindMatches(const std::aligned_vector<Eigen::Vector3d>& points,
      const std::vector<cv::Mat>& descriptors,
      const cv::DescriptorMatcher& descriptorMatcher,
      const double matchingDistanceThreshold,
      const size_t matchingNeighborhoodThreshold
    ) const;

    inline void SetMatchedKeyPoint( size_t index ) const
    { imageFeatures_.SetMatchedKeyPoint( index ); }

    inline void GetUnmatchedKeyPoints(std::vector<cv::KeyPoint>& keyPoints, cv::Mat& descriptors, std::vector<size_t>& indexes) const
    { return imageFeatures_.GetUnmatchedKeyPoints(keyPoints, descriptors, indexes); }

  private:

    std::vector<cv::Point2d> ComputeProjections(const std::vector<cv::Point3d>& points);

  private:

    // the pose of the camera
    Camera camera_;

    // imageFeatures for matching during refinement
    // It's mutable because it marks matched features
    // to boost matching performance.
    // TODO Maybe there should be mutable some members of imageFeatures and no the whole object
    ImageFeatures imageFeatures_;
};
