cmake_minimum_required(VERSION 2.8.3)
project( sptam )

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules)

# declare cache entry parameters
SET(SHOW_TRACKED_FRAMES OFF CACHE BOOL "Enable/Disable OpenCV frame visualization for the tracker.")
SET(SHOW_PROFILING ON CACHE BOOL "Enable/Disable Profiling of each step.")

if( SHOW_TRACKED_FRAMES )
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DSHOW_TRACKED_FRAMES")
endif()

if( SHOW_PROFILING )
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DSHOW_PROFILING")
endif()

## Find catkin macros and libraries
find_package( catkin REQUIRED COMPONENTS
  roscpp
  cmake_modules
  std_msgs
  sensor_msgs
  geometry_msgs
  cv_bridge
  message_filters
  image_geometry
  pcl_ros
  nav_msgs
  tf_conversions
  image_transport
)

# Find Boost Library
find_package(Boost COMPONENTS thread system regex REQUIRED)
include_directories(${BOOST_INCLUDE_DIR})

# Find yaml-cpp Library
find_package(PkgConfig)
pkg_check_modules(YamlCpp yaml-cpp)

# Find OpenCV library
find_package(OpenCV REQUIRED)

# Find PCL library
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# Find Suitesparse library
find_package(SuiteSparse REQUIRED)
include_directories(${CHOLMOD_INCLUDE_DIR})

# Find G2O Library
find_package(G2O REQUIRED)
# select required components
SET(G2O_LIBRARIES ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${G2O_SOLVER_CSPARSE} ${G2O_SOLVER_CSPARSE_EXTENSION} ${G2O_TYPES_SBA})

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sptam
   CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs cv_bridge message_filters image_geometry pcl_ros nav_msgs tf_conversions
#  DEPENDS system_lib
)

###########
## Build ##
###########

# Build the sptam library
add_subdirectory( src/sptam )

set( SPTAM_LIBRARIES
  ${OpenCV_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  ${catkin_LIBRARIES}
  ${G2O_LIBRARIES}
  ${SUITESPARSE_LIBRARIES}
  ${YamlCpp_LIBRARIES}
  cxsparse
  sptam
)

# Build the sptam node
add_subdirectory( src/ros )

#############
## Install ##
#############

#TODO

#############
## Testing ##
#############

#TODO
