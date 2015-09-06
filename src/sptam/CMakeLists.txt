cmake_minimum_required(VERSION 2.8.3)
project( sptam )

# declare cache entry parameters
SET(SHOW_POINT_CLOUD OFF)
SET(SHOW_TRACKED_FRAMES OFF CACHE BOOL "Enable/Disable OpenCV frame visualization for the tracker.")
SET(SHOW_PROFILING ON CACHE BOOL "Enable/Disable Profiling of each step.")

# Enable most warnings
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-deprecated-declarations")

# Enable C++11 support
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# maximize optimization
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O4")


find_package(OpenCV REQUIRED)


# Set CMake flags in the compiler

if( SHOW_TRACKED_FRAMES )
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DSHOW_TRACKED_FRAMES")
endif()

if( SHOW_PROFILING )
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DSHOW_PROFILING")
endif()

if( USE_ODOMETRY )
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_ODOMETRY")
endif()

# List of files to compile
FILE(GLOB SLAM_SRCS
 *.cpp
 utils/*.cpp
)

# Build the sptam library
add_library( sptam  ${SLAM_SRCS})
target_link_libraries( sptam ${OpenCV_LIBRARIES})
