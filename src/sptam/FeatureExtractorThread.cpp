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

#include "FeatureExtractorThread.hpp"

#ifdef SHOW_PROFILING
  #include "utils/Profiler.hpp"
  #include "utils/Time.hpp"
#endif // SHOW_PROFILING

FeatureExtractorThread::FeatureExtractorThread(
  const cv::Mat& image,
  const cv::FeatureDetector& featureDetector,
  const cv::DescriptorExtractor& descriptorExtractor
)
  : image_( image )
  , featureDetector_( featureDetector )
  , descriptorExtractor_( descriptorExtractor )
  , featureExtractorThread_(&FeatureExtractorThread::Extract, this) // call thread
{}

void FeatureExtractorThread::Extract()
{
  #ifdef SHOW_PROFILING
    double startStep, endStep;
    startStep = GetSeg();
  #endif

  featureDetector_.detect(image_, keyPoints_);

  #ifdef SHOW_PROFILING
    endStep = GetSeg();
    WriteToLog(" tk FeatureDetection: ", startStep, endStep);
    startStep = GetSeg();
  #endif

  descriptorExtractor_.compute(image_, keyPoints_, descriptors_);

  #ifdef SHOW_PROFILING
    endStep = GetSeg();
    WriteToLog(" tk DescriptorExtraction: ", startStep, endStep);
  #endif
}
