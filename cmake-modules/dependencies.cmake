# Find Boost Library
find_package(Boost COMPONENTS system thread regex REQUIRED)
#~ include_directories(${BOOST_INCLUDE_DIR})

# Find yaml-cpp Library
find_package(PkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)

# Find Eigen3 Library
# Since it is header-only and it won't be linked,
# we have to explicitly add the include directories.
find_package(Eigen REQUIRED)
include_directories(${EIGEN_INCLUDE_DIRS})

# Find OpenCV library
#find_package(OpenCV REQUIRED xfeatures2d features2d core)

###############################################################
# BUG FIX for OpenCV 3.3.1 in ROS Kinetic (see: https://github.com/ros-perception/vision_opencv/issues/193)
###############################################################

find_package(OpenCV 3 REQUIRED)
if (${OpenCV_VERSION} MATCHES "3.3.1")
  foreach(__cvcomponent ${OpenCV_LIB_COMPONENTS})
    set (__original_cvcomponent ${__cvcomponent})
    if(NOT __cvcomponent MATCHES "^opencv_")
      set(__cvcomponent opencv_${__cvcomponent})
    endif()
    if (TARGET ${__cvcomponent})
      set_target_properties(${__cvcomponent} PROPERTIES
          MAP_IMPORTED_CONFIG_DEBUG ""
          MAP_IMPORTED_CONFIG_RELEASE ""
          MAP_IMPORTED_CONFIG_RELWITHDEBINFO ""
          MAP_IMPORTED_CONFIG_MINSIZEREL ""
      )
    endif()
  endforeach(__cvcomponent)
endif()

###############################################################


# Find Suitesparse library
find_package(SuiteSparse REQUIRED)

# Find G2O Library
find_package(G2O REQUIRED)
# select required components
set(G2O_LIBRARIES ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${G2O_SOLVER_CSPARSE} ${G2O_SOLVER_CSPARSE_EXTENSION} ${G2O_TYPES_SBA} ${G2O_TYPES_SLAM3D})

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
  include_directories(${OPENGV_INCLUDE_DIR})
  
  # List of files to compile
  file(GLOB LC_SRCS src/sptam/loopclosing/*.cpp src/sptam/loopclosing/detectors/*.cpp)
endif()
