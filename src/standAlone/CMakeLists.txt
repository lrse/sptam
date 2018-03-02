cmake_minimum_required(VERSION 3.0.2)
project( sptam )

# X11 library is required for multithreaded visualization in OpenCV
if( SHOW_TRACKED_FRAMES )
  find_package(X11 REQUIRED)
  include_directories(${X11_INCLUDE_DIRS})
endif()

# Link to standard threading library
if( NOT SINGLE_THREAD )
  set(THREAD_LIBS pthread)
endif()

# Compile frame generator library
file(GLOB frameGeneratorFiles FrameGenerator/*.cpp)
add_library(frameGenerator ${frameGeneratorFiles})
target_link_libraries(frameGenerator ${OpenCV_LIBS})

file(GLOB STANDALONE_SRCS StereoFeatureExtractor.cpp KITTIGroundTruth.cpp SptamWrapper.cpp utils/ProgramOptions.cpp Timestamps.cpp)

add_executable(sptam-stereo sptam-stereo.cpp ${STANDALONE_SRCS} ${SLAM_SRCS})
target_link_libraries(sptam-stereo sptam frameGenerator ${X11_LIBRARIES} boost_program_options ${THREAD_LIBS})
