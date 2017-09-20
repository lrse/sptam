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

#include "SptamWrapperThread.hpp"
#include <boost/chrono.hpp>
#include <boost/thread/thread.hpp>

#include "../sptam/utils/Profiler.hpp"
#include "../sptam/utils/Logger.hpp"

SptamWrapperThread::SptamWrapperThread(
  const CameraParameters &cameraParametersLeft,
  const CameraParameters &cameraParametersRight,
  const double stereo_baseline, const RowMatcher &rowMatcher,
  const MapMaker::Parameters &params,
  std::unique_ptr<PosePredictor> motionModel,
  const size_t imageBeginIndex
)
  : SptamWrapper(cameraParametersLeft, cameraParametersRight, stereo_baseline, rowMatcher, params, std::move(motionModel), imageBeginIndex)
  , stop_( false )
{
  thread_ = std::thread(&SptamWrapperThread::run, this);
}

void SptamWrapperThread::Add(const ImageFeatures &imageFeaturesLeft, const ImageFeatures &imageFeaturesRight,
  const cv::Mat& imageLeft, const cv::Mat& imageRight)
{

  std::unique_ptr<StereoImageFeatures> stereoImageFeatures(new StereoImageFeatures( imageFeaturesLeft, imageFeaturesRight, imageLeft, imageRight));

  queue_.push( stereoImageFeatures.release() );

  #ifdef SHOW_PROFILING
    WriteToLog(" tk queueParallelSize: ", queue_.size());
  #endif
}

void SptamWrapperThread::run() {

  while( not stop_ ) {


    StereoImageFeatures* stereoImageFeatures_aux;
    /*bool moreData = */queue_.waitAndPop( stereoImageFeatures_aux );
    std::unique_ptr<StereoImageFeatures> stereoImageFeatures(stereoImageFeatures_aux);

    ImageFeatures &imageFeaturesLeft = stereoImageFeatures->imageFeaturesLeft;
    ImageFeatures &imageFeaturesRight = stereoImageFeatures->imageFeaturesRight;
    const cv::Mat &imageLeft = stereoImageFeatures->imageLeft;
    const cv::Mat &imageRight = stereoImageFeatures->imageRight;

    #ifdef SHOW_PROFILING
      sptam::Timer t_tracking;
      t_tracking.start();
    #endif // SHOW_PROFILING

    cv::Point3d estimatedCameraPosition;
    cv::Vec4d estimatedCameraOrientation;

    CameraPose currentCameraPose = GetCurrentCameraPose();

    // Check is there is no initial map create it
    if (not isMapInitialized_)
    {
      // HACK: cuando se utiliza el Ground-Truth hace que avance la pose adecuadamente (en el caso de que la primera imagen no sirva para inicializar el mapa)
      // Cuando se utiliza motion model no hace nada
      if ( currentFrameIndex_ != 0 ) {
        motionModel_->PredictNextCameraPose( estimatedCameraPosition, estimatedCameraOrientation );
        currentCameraPose = CameraPose(estimatedCameraPosition, estimatedCameraOrientation);
      }

      // Initialize MapMaker
      isMapInitialized_ = sptam_.init(currentCameraPose, imageFeaturesLeft, imageFeaturesRight);
    }
    // if there is an initial map then run SPTAM
    else
    {
      motionModel_->PredictNextCameraPose( estimatedCameraPosition, estimatedCameraOrientation );
      const CameraPose estimatedCameraPose( estimatedCameraPosition, estimatedCameraOrientation );


      // Main process
      currentCameraPose = sptam_.track( estimatedCameraPose,
                                       imageFeaturesLeft,
                                       imageFeaturesRight,
                                       imageLeft,
                                       imageRight);
    } // else


    /*
     * Print pose transformation as required by the KITTI dataset
     * to compare with ground truth. Print in row major order:
     *
     * O00 O01 O02 P0
     * O10 O11 O12 P1
     * O20 O21 O22 P2
     *
     * Where O:3x3 is the orientation matrix and P:3x1 is the position
     * of the left (grayscale) camera.
     */
    #ifdef SHOW_PROFILING
      writePoseToLog("TRACKED_FRAME_POSE", currentFrameIndex_, currentCameraPose.GetPosition(), currentCameraPose.GetOrientationMatrix());
    #endif

    #ifdef SHOW_PROFILING
      t_tracking.stop();
      WriteToLog(" tk trackingtotal: ", t_tracking);
    #endif

    // Update motion model
    motionModel_->UpdateCameraPose( currentCameraPose.GetPosition(), currentCameraPose.GetOrientationQuaternion() );
    currentFrameIndex_++;

    SetCurrentCameraPose(currentCameraPose);


  }
}

void SptamWrapperThread::Stop()
{

  stop_ = true;

  sptam_.stop();
  thread_.join();
}
