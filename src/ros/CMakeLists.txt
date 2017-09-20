cmake_minimum_required(VERSION 3.0.2)
project( sptam )

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

# check if SPTAM_LIBRARIES is defined
if( SPTAM_LIBRARIES )

  add_library(sptam_nodelet sptam_nodelet.cpp stereo_driver.cpp base_driver.cpp utils/tf_utils.cpp utils/opencv_parsers.cpp ${LC_SRC})

  target_link_libraries(sptam_nodelet ${SPTAM_LIBRARIES})

  add_executable(sptam_node sptam_node.cpp)
  target_link_libraries( sptam_node ${SPTAM_LIBRARIES})

endif()
