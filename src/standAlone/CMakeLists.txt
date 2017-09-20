CMAKE_MINIMUM_REQUIRED(VERSION 2.8 FATAL_ERROR)
PROJECT( sptam )

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/../../cmake-modules)

# include macros for program options
include( options )

# Find Boost Library
FIND_PACKAGE(Boost COMPONENTS system thread regex REQUIRED)
INCLUDE_DIRECTORIES(${BOOST_INCLUDE_DIR})
# Preserve project Boost required libraries
# This is because of a bug in FindPCL.cmake which overwrites the BOOST
# libs and includes with its own dependencies, instead of merging them.
# fix: http://stackoverflow.com/questions/19720639/handling-multiple-find-package-calls-in-cmake
SET(Boost_PROJECT_LIBRARIES ${Boost_LIBRARIES})

# Find yaml-cpp Library
find_package(PkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)

# Find Eigen3 Library
find_package(Eigen REQUIRED)
INCLUDE_DIRECTORIES(${EIGEN_INCLUDE_DIRS})

# Find OpenCV library
FIND_PACKAGE(OpenCV REQUIRED)
#FIND_PACKAGE(OpenCV 2 REQUIRED)

message(${OpenCV_LIBS})

# Find Suitesparse library
find_package(SuiteSparse REQUIRED)
include_directories(${CSPARSE_INCLUDE_DIR})
include_directories(${CHOLMOD_INCLUDE_DIR})

# Find G2O Library
find_package(G2O REQUIRED)
# select required components
SET(G2O_LIBRARIES ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${G2O_SOLVER_CSPARSE} ${G2O_SOLVER_CSPARSE_EXTENSION} ${G2O_TYPES_SBA} ${G2O_TYPES_SLAM3D})
include_directories(${G2O_INCLUDE_DIRS})

## Because EKF uses ros::Time
find_package( catkin REQUIRED COMPONENTS roscpp )
include_directories(${catkin_INCLUDE_DIRS})

#if tracker debugging is enabled, compile the visualization library
if ( SHOW_TRACKED_FRAMES )
  # Find X11 library required for multithreaded visualization in OpenCV
  FIND_PACKAGE(X11 REQUIRED)
  INCLUDE_DIRECTORIES(${X11_INCLUDE_DIRS})
endif()

# Find libraries required by LoopClosing module
if( USE_LOOPCLOSURE )
  # DLoopDetector library (must be properly installed from github repos)
  find_package(DLib REQUIRED)
  find_package(DBoW2 REQUIRED)
  find_package(DLoopDetector REQUIRED)
  include_directories(${DLib_INCLUDE_DIRS} ${DBoW2_INCLUDE_DIRS} ${DLoopDetector_INCLUDE_DIRS})
  set(DLD_LIBRARIES ${DLib_LIBRARIES} ${DBoW2_LIBRARIES})

  # Find OpenGV
  find_package(OpenGV REQUIRED)
  INCLUDE_DIRECTORIES(${OPENGV_INCLUDE_DIR})

  # List of files to compile LoopClosing module
  FILE(GLOB LC_SRC ../sptam/loopclosing/*.cpp ../sptam/loopclosing/detectors/*.cpp)

  # If USE_LOOPCLOSURE isnt set, then LC_SRC will be "blank"
endif()

# SPTAM source files location
SET(SPTAM_SRC ../sptam)

# List of common SLAM_SRCS files to compile
FILE(GLOB SLAM_SRCS ${SPTAM_SRC}/*.cpp ${SPTAM_SRC}/utils/*.cpp)

# Compile the tracker visualization library
if ( SHOW_TRACKED_FRAMES )
  set(SLAM_SRCS ${SLAM_SRCS} ${SPTAM_SRC}/utils/draw/Draw.cpp)
endif()

# Compile the profiling library
if ( SHOW_PROFILING )
  set(SLAM_SRCS ${SLAM_SRCS} ${SPTAM_SRC}/utils/log/Logger.cpp)
endif()

# Link to threading library
if ( NOT SINGLE_THREAD )
  set(THREAD_LIBS pthread)
endif()

# Compile frame generator library
FILE(GLOB frameGeneratorFiles FrameGenerator/*.cpp)
add_library(frameGenerator  ${frameGeneratorFiles} )
# For OpenCV 2 frameGenerator library must be linked individually
target_link_libraries(frameGenerator ${OpenCV_LIBS})
set(SPTAM_LIBS ${SPTAM_LIBS} frameGenerator)


FILE(GLOB standAlone StereoFeatureExtractor.cpp KITTIGroundTruth.cpp SptamWrapper.cpp utils/ProgramOptions.cpp Timestamps.cpp)

ADD_EXECUTABLE(sptam-stereo sptam-stereo.cpp ${standAlone} ${SLAM_SRCS} ${LC_SRC})
TARGET_LINK_LIBRARIES(sptam-stereo ${X11_LIBRARIES} ${SPTAM_LIBS} ${PCL_LIBRARIES} ${OpenCV_LIBS} ${Boost_LIBRARIES} boost_program_options ${Boost_PROJECT_LIBRARIES} ${catkin_LIBRARIES} ${G2O_LIBRARIES} ${YamlCpp_LIBRARIES} ${SUITESPARSE_LIBRARIES} ${CHOLMOD_LIBRARIES} cxsparse ${THREAD_LIBS} ${DLD_LIBRARIES} ${OPENGV_LIBRARY} ${TBB_LIBRARIES})
