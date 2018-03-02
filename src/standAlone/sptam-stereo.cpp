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

#include "SptamWrapper.hpp"
#include "KITTIGroundTruth.hpp"


#include "opencv2/core/version.hpp"
#if CV_MAJOR_VERSION == 2
  #include <opencv2/highgui/highgui.hpp>
  #include "configuration_parser_opencv2.hpp"
#elif CV_MAJOR_VERSION == 3
  #include <opencv2/highgui.hpp>
  #include "configuration_parser_opencv3.hpp"
#endif

#include "../sptam/utils/macros.hpp"
#include "../sptam/utils/projective_math.hpp"

#include "Timestamps.hpp"
#include "utils/ProgramOptions.hpp"
#include "FrameGenerator/FrameGeneratorFactory.hpp"

#include "StereoImageFeatures.hpp"
#include "../sptam/FeatureExtractorThread.hpp"

#include "../sptam/MotionModel.hpp"

#include <signal.h>

#ifdef USE_LOOPCLOSURE
  #include "../sptam/loopclosing/LoopClosing.hpp"
  #include "../sptam/loopclosing/LCDetector.hpp"
  #include "../sptam/loopclosing/detectors/DLDLoopDetector.hpp"
#endif

#ifdef SHOW_TRACKED_FRAMES
  #include "../sptam/utils/draw/Draw.hpp"
  #include <X11/Xlib.h> // XInitThreads()
#endif // SHOW_TRACKED_FRAMES

#ifdef SHOW_POINT_CLOUD
  #include "gui/PointCloud.hpp"
#endif // SHOW_POINT_CLOUD

#ifdef SHOW_PROFILING
  #include "../sptam/utils/log/Profiler.hpp"
  #include "../sptam/utils/log/ScopedProfiler.hpp"
#endif // SHOW_PROFILING

#define INITIAL_POSE_COVARIANCE Eigen::Matrix6d::Identity() * 1e-6

// Used to smoothly finish the process in case of an interruption
bool programMustEnd = false;
// handler function called by interruptions signals
void interrupt_signal(int s) { programMustEnd = true; }

template<typename T>
void loadParameter(const YAML::Node& config, const std::string key, T& ret, const T& default_value)
{
  try {
    ret = config[ key ].as<T>();
  } catch(YAML::KeyNotFound& e) {
    ret = default_value;
  }
}

inline CameraPose getCurrentCameraPose(const std::shared_ptr<PosePredictor>& motion_model)
{
  Eigen::Vector3d currentCameraPosition;
  Eigen::Quaterniond currentCameraOrientation;
  Eigen::Matrix6d covariance;
  motion_model->currentPose(currentCameraPosition, currentCameraOrientation, covariance);
  return CameraPose(currentCameraPosition, currentCameraOrientation, covariance);
}

int main(int argc, char* argv[])
{
#if CV_MAJOR_VERSION == 2
  // Load Nonfree OpenCv modules (SURF,SIFT)
  cv::initModule_nonfree();
#endif // CV_MAJOR_VERSION

#ifdef SHOW_TRACKED_FRAMES
  // XInitThreads function allows to show images in OpenCV in every thread using cv::imshow()
  XInitThreads();
#endif // SHOW_TRACKED_FRAMES

#ifdef SHOW_PROFILING
  // Show doubles with presicion with cout
  std::cout << std::fixed;
#endif // shOW_PROFILING

  size_t imageBeginIndex;
  std::string leftImages, rightImages, parametersFileYML, calibration_file, timestampsFile, groundTruthPath, imagesSourceType;

  /** Define the program options */

  ProgramOptions program_options( argv[0] );
  program_options.addPositionalArgument("configuration", "configuration file with SPTAM parameters.", parametersFileYML);
  program_options.addPositionalArgument("calibration", "camera calibration file.", calibration_file);
  program_options.addPositionalArgument("left-images", "left camera source (image directory, streaming device or image list file).", leftImages);
  program_options.addPositionalArgument("right-images", "right camera source (image directory, streaming device or image list file).", rightImages);
  program_options.addPositionalArgument("images-source", "source type: 'dir', 'cam' or 'list'.", imagesSourceType);

  // TODO
  // add optional ini frame
  // add optional max frame
  program_options.addOptionalArgument("timestamps", "file containing the timestamps for each image frame.", timestampsFile);
  program_options.addOptionalArgument("grnd-poses", "Use ground truth poses file for pose prediction step.", groundTruthPath);
  program_options.addOptionalArgument("from", "Start the sequence from a certain frame.", imageBeginIndex);
  program_options.addOptionalArgumentFlag("help", "show help.");

  /** Parse the program options */

  try
  {
    // may throw
    program_options.parse( argc, argv );

    // if count 'help' show help and exit
    if (program_options.count("help") ) {
      std::cerr << program_options << std::endl;
      return 0;
    }

    // throws on error, so do after help in case there are any problems.
    program_options.notify();
  } 
  catch(boost::program_options::error& e)
  {
    std::cerr << "ERROR: " << e.what() << std::endl << std::endl;
    std::cerr << program_options << std::endl;
    return EXIT_FAILURE;
  }

  /** Load program parameters from configuration file */

  // Load S-PTAM parameters
  Parameters sptam_params;
  cv::Ptr<cv::FeatureDetector> feature_detector_left, feature_detector_right;
  cv::Ptr<cv::DescriptorExtractor> descriptor_extractor_left, descriptor_extractor_right;
  double frustum_near_plane_distance_, frustum_far_plane_distance_;

  #ifdef USE_LOOPCLOSURE
  std::unique_ptr<LCDetector> loop_detector(nullptr);
  #endif

  try
  {
    YAML::Node config = YAML::LoadFile( parametersFileYML );

    loadParameter(config, "FrustumNearPlaneDist", frustum_near_plane_distance_, 0.1);
    loadParameter(config, "FrustumFarPlaneDist", frustum_far_plane_distance_, 1000.0);

    for (auto it : config)
    {
      std::string key = it.first.as<std::string>();
      if (key == "MatchingCellSize") sptam_params.matchingCellSize = it.second.as<size_t>();
      else if (key == "MatchingNeighborhood") sptam_params.matchingNeighborhoodThreshold = it.second.as<size_t>();
      else if (key == "MatchingDistance") sptam_params.matchingDistanceThreshold = it.second.as<double>();
      else if (key == "EpipolarDistance") sptam_params.epipolarDistanceThreshold = it.second.as<size_t>();
      else if (key == "BundleAdjustmentActiveKeyframes") sptam_params.nKeyFramesToAdjustByLocal = it.second.as<size_t>();
      else if (key == "maxIterationsLocal") sptam_params.maxIterationsLocal = it.second.as<size_t>();
      else if (key == "minimumTrackedPointsRatio") sptam_params.minimumTrackedPointsRatio = it.second.as<double>();
    }

    if (config["FeatureDetector"]["nFeatures"].IsDefined())
      sptam_params.nFeatures = config["FeatureDetector"]["nFeatures"].as<size_t>();

    feature_detector_left = loadFeatureDetector( config["FeatureDetector"] );
    feature_detector_right = loadFeatureDetector( config["FeatureDetector"] );

    descriptor_extractor_left = loadDescriptorExtractor( config["DescriptorExtractor"] );
    descriptor_extractor_right = loadDescriptorExtractor( config["DescriptorExtractor"] );

    sptam_params.descriptorMatcher = loadDescriptorMatcher( config["DescriptorMatcher"] );

    #ifdef USE_LOOPCLOSURE
    if (config["LoopDetectorVocabulary"])
      sptam_params.loopDetectorVocabulary = config["LoopDetectorVocabulary"].as<std::string>();

    /* Brief detector its the only one implemented for the moment */
    if(config["DescriptorExtractor"]["Name"].as<std::string>().compare("BRIEF") == 0){
      DLDLoopDetector<DBoW2::FBrief>::Parameters lcd_param; // Detector parameters by default
      std::cout << "Initializing Loop Detector, loading vocabulary" << std::endl;
      loop_detector.reset(new DLDLoopDetector<DBoW2::FBrief>(sptam_params.loopDetectorVocabulary, lcd_param));
    } else if(config["DescriptorExtractor"]["Name"].as<std::string>().compare("BRISK") == 0){
      DLDLoopDetector<DBoW2::FBRISK>::Parameters lcd_param; // Detector parameters by default
      std::cout << "Initializing Loop Detector, loading vocabulary" << std::endl;
      loop_detector.reset(new DLDLoopDetector<DBoW2::FBRISK>(sptam_params.loopDetectorVocabulary, lcd_param));
    }
    #endif
  }
  catch(YAML::BadFile& e)
  {
    std::cerr << "Could not open configuration file " << parametersFileYML << std::endl;
    return EXIT_FAILURE;
  }
  catch(YAML::ParserException& e)
  {
    std::cerr << "Could not parse configuration file " << parametersFileYML << ". " << e.what() << std::endl;
    return EXIT_FAILURE;
  }

  // write configuration to log file
  #ifdef SHOW_PROFILING
    Logger::Write( "#   matchingCellSize: " + std::to_string( sptam_params.matchingCellSize ) + "\n" );
    Logger::Write( "#   matchingNeighborhoodThreshold: " + std::to_string( sptam_params.matchingNeighborhoodThreshold ) + "\n" );
    Logger::Write( "#   matchingDistanceThreshold: " + std::to_string( sptam_params.matchingDistanceThreshold ) + "\n" );
    Logger::Write( "#   epipolarDistanceThreshold: " + std::to_string( sptam_params.epipolarDistanceThreshold ) + "\n" );
    Logger::Write( "#   FrustumNearPlaneDist: " + std::to_string( frustum_near_plane_distance_ ) + "\n" );
    Logger::Write( "#   FrustumFarPlaneDist: " + std::to_string( frustum_far_plane_distance_ ) + "\n" );
    Logger::Write( "#   BundleAdjustmentActiveKeyframes: " + std::to_string( sptam_params.nKeyFramesToAdjustByLocal ) + "\n" );
    #ifdef USE_LOOPCLOSURE
    Logger::Write( "#   LoopDetectorVocabulary: " + sptam_params.loopDetectorVocabulary + "\n" );
    #endif
  #endif

  // write sequence image source to log file
  #ifdef SHOW_PROFILING
    Logger::Write( "#   source_left_images: " + leftImages + "\n" );
    Logger::Write( "#   source_right_images: " + rightImages + "\n" );
  #endif

  RowMatcher rowMatcher(sptam_params.matchingDistanceThreshold, sptam_params.descriptorMatcher, sptam_params.epipolarDistanceThreshold);

  const CameraParameters cameraCalibration = loadCameraCalibration(calibration_file, frustum_near_plane_distance_, frustum_far_plane_distance_);
  CameraParameters cameraParametersLeft = cameraCalibration;
  CameraParameters cameraParametersRight = cameraCalibration;

  /** Read source images or video stream */

  if ( not program_options.count("from") )
    imageBeginIndex = 0;

  const size_t imageEndIndex = std::numeric_limits<size_t>::max(); // std::numeric_limits<size_t>::max() the whole sequence;

  std::unique_ptr<IFrameGenerator> frameGeneratorLeft = createFrameGenerator(leftImages, imagesSourceType, imageBeginIndex, imageEndIndex);
  std::unique_ptr<IFrameGenerator> frameGeneratorRight = createFrameGenerator(rightImages, imagesSourceType, imageBeginIndex, imageEndIndex);

  /** Subscribe interruption handlers */

  signal(SIGINT, &interrupt_signal);
  signal(SIGTERM, &interrupt_signal);

  /** Initialize pose prediction model */

  const bool useMotionModel = not program_options.count("grnd-poses");
  const bool useTimestamps = program_options.count("timestamps");

  std::shared_ptr<PosePredictor> motionModel(
    useMotionModel
    // Initialize Current Camera Pose With Left Canonical Position
    ? (PosePredictor*) new MotionModel( ros::Time(0), Eigen::Vector3d::Zero(), Eigen::Quaterniond::Identity(), INITIAL_POSE_COVARIANCE )
    : (PosePredictor*) new KITTIGroundTruth( groundTruthPath )
  );

  // Get First Frames for use in the loop
  cv::Mat imageLeft, imageRight;
  bool hasNextFrame = frameGeneratorLeft->getNextFrame( imageLeft ) and frameGeneratorRight->getNextFrame( imageRight );

  // Create SPTAM wrapper
  SptamWrapper sptamWrapper ( cameraParametersLeft, cameraParametersRight, cameraCalibration.baseline(),
    rowMatcher, sptam_params, motionModel, imageBeginIndex);

  #ifdef USE_LOOPCLOSURE
  sptamWrapper.setLoopClosing(loop_detector);
  #endif

  #ifdef SHOW_POINT_CLOUD
    const sptam::Map &map = sptamWrapper.GetMap();
    PointCloud pointCloud(getCurrentCameraPose( motionModel ), map,
                          cameraParametersLeft.horizontalFov(), cameraParametersLeft.verticalFov(),
                          frustum_near_plane_distance_, frustum_far_plane_distance_);
  #endif

  // TODO hardcoded rate in case of no timestamp. Parametrize.
  Timestamps timestamps = useTimestamps ? Timestamps(timestampsFile, imageBeginIndex) : Timestamps(0.1, imageBeginIndex);

  // KITTI images count from 0, but all scripts and rosbag expect it to start at 1.
  size_t frame_number = imageBeginIndex + 1;

  // Main loop
  while( hasNextFrame and not programMustEnd )
  {
    /** Compute time remainder so we don't go too fast, sleep if necessary */
    ros::Time current_time = timestamps.getNextWhenReady();

    #ifdef SHOW_PROFILING
    {
      writeToLog("FRAME_TIMESTAMP", frame_number, current_time.sec, current_time.nsec);

      sptam::ScopedProfiler timer(" tk trackingtotal: ");

      sptam::Timer t_extraction;
      t_extraction.start();
    #endif

    // Detect features and extract descriptors from new frames

    FeatureExtractorThread featureExtractorThreadLeft(imageLeft, feature_detector_left, descriptor_extractor_left, sptam_params.nFeatures);
    FeatureExtractorThread featureExtractorThreadRight(imageRight, feature_detector_right, descriptor_extractor_right, sptam_params.nFeatures);

    featureExtractorThreadLeft.WaitUntilFinished();
    const std::vector<cv::KeyPoint>& keyPointsLeft = featureExtractorThreadLeft.GetKeyPoints();
    const cv::Mat& descriptorsLeft = featureExtractorThreadLeft.GetDescriptors();

    featureExtractorThreadRight.WaitUntilFinished();
    const std::vector<cv::KeyPoint>& keyPointsRight = featureExtractorThreadRight.GetKeyPoints();
    const cv::Mat& descriptorsRight = featureExtractorThreadRight.GetDescriptors();

    ImageFeatures imageFeaturesLeft(imageLeft.size(), keyPointsLeft, descriptorsLeft, sptam_params.matchingCellSize);
    ImageFeatures imageFeaturesRight(imageRight.size(), keyPointsRight, descriptorsRight, sptam_params.matchingCellSize);

    std::unique_ptr<StereoImageFeatures> stereoImageFeatures(new StereoImageFeatures(imageFeaturesLeft,imageFeaturesRight, imageLeft, imageRight) );

    #ifdef SHOW_PROFILING
      t_extraction.stop();
      WriteToLog(" tk extraction: ", t_extraction);
      WriteToLog(" tk ExtractedPoints: ", keyPointsLeft.size() + keyPointsRight.size());
    #endif

    sptamWrapper.Add(frame_number, current_time, std::move(stereoImageFeatures));

    #ifdef SHOW_PROFILING
    }
    #endif

    #ifdef SHOW_POINT_CLOUD

      pointCloud.SetCameraPose( getCurrentCameraPose( motionModel ) );

    #endif // SHOW_POINT_CLOUD

    // Get Next Frames
    hasNextFrame = frameGeneratorLeft->getNextFrame( imageLeft ) and frameGeneratorRight->getNextFrame( imageRight );
    frame_number++;
  }

  #ifdef SHOW_POINT_CLOUD
    // Create PLY file to be read by meshlab software
    CreatePLYFile( map );
  #endif // SHOW_POINT_CLOUD

  std::cout << "Wait for stop..." << std::endl;

  // Wait the mapper quits the main loop and joins.
  sptamWrapper.Stop();

  #ifdef SHOW_POINT_CLOUD
    // Stop visualizer Thread
    pointCloud.Stop();
  #endif

  #ifdef SHOW_PROFILING
/*
    for ( const auto& mapPoint : sptamWrapper.GetMapPoints() ) {
      WriteToLog( " tk MeasurementCount: ", mapPoint.measurements().size() );
    }

    for ( const auto& keyFrame : sptamWrapper.GetKeyFrames() ) {
      CameraPose keyFramePose = keyFrame.GetCameraPose();
      writePoseToLog("FINAL_FRAME_POSE", keyFrame.GetId(), keyFramePose.GetPosition(), keyFramePose.GetOrientationMatrix(), keyFramePose.covariance());
    }
*/
/*
    // dump poses file
    {
      std::ofstream out( "sptam_poses.dat" );
      for ( const auto& keyframe : sptamWrapper.GetMap().getKeyframes() ) {
        CameraPose keyframePose = keyframe->GetCameraPose();
        __poseToStream__(out, keyframePose.GetPosition(), keyframePose.GetOrientationMatrix(), keyframePose.covariance()) << std::endl;
      }
      out.close();
    }

    // dump map file
    {
      std::ofstream out( "sptam_points.dat" );
      for ( const auto& mapPoint : sptamWrapper.GetMap().getMapPoints() ) {
        __pointToStream__(out, mapPoint->GetPosition(), mapPoint->covariance()) << std::endl;
      }
      out.close();
    }
*/
  #endif

  std::cout << "Stop succesfull!" << std::endl;

  #ifdef SHOW_TRACKED_FRAMES
  cv::destroyAllWindows();
  #endif

  return 0;
}
