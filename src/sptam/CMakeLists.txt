cmake_minimum_required(VERSION 3.0.2)
project( sptam )

# Find OpenCV library
FIND_PACKAGE(OpenCV REQUIRED)
#FIND_PACKAGE(OpenCV 2 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

# List of files to compile
FILE(GLOB SLAM_SRCS *.cpp utils/*.cpp)

# Find libraries required by LoopClosing module
if( USE_LOOPCLOSURE )
  # DLoopDetector library (must be properly installed from github repos)
  find_package(DLib REQUIRED)
  find_package(DBoW2 REQUIRED)
  find_package(DLoopDetector REQUIRED)
  include_directories(${DLib_INCLUDE_DIRS} ${DBoW2_INCLUDE_DIRS} ${DLoopDetector_INCLUDE_DIRS})
  set(DLD_LIBRARIES ${DLib_LIBRARIES} ${DBoW2_LIBRARIES}) # DLoopDetector its just a header
  
  # Find OpenGV
  find_package(OpenGV REQUIRED)
  INCLUDE_DIRECTORIES(${OPENGV_INCLUDE_DIR})
  
  # List of files to compile
  FILE(GLOB LC_SRCS loopclosing/*.cpp)
  set(SLAM_SRCS ${SLAM_SRCS} ${LC_SRC})
  
  # if LC isnt defined, DLD_LIBRARIES OPENGV_LIBRARY are "blank"
endif()

# Compile the tracker visualization library
if ( SHOW_TRACKED_FRAMES )
  set(SLAM_SRCS ${SLAM_SRCS} utils/draw/Draw.cpp)
endif()

# Compile the profiling library
if ( SHOW_PROFILING )
  set(SLAM_SRCS ${SLAM_SRCS} utils/log/Logger.cpp)
endif()

# Build the sptam library
add_library( sptam  ${SLAM_SRCS} )
target_link_libraries( sptam ${OpenCV_LIBRARIES} ${DLD_LIBRARIES} ${OPENGV_LIBRARY} ${TBB_LIBRARIES})
