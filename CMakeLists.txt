cmake_minimum_required(VERSION 3.0.2)
project( sptam )

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/cmake-modules)

# include macros for program options
include( options )

# include finds for common program dependencies
include( dependencies )

# Build the sptam library
add_subdirectory( src/sptam )

# build unit tests
enable_testing()
add_subdirectory( src/tests )

# When building with ROS
if( CATKIN_DEVEL_PREFIX )

set(BUILDING_ROS ON)
add_definitions(-DBUILDING_ROS)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
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
  tf2
  tf2_geometry_msgs
  tf2_ros
  image_transport
  nodelet
)
include_directories(${catkin_INCLUDE_DIRS})

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
  CATKIN_DEPENDS roscpp std_msgs sensor_msgs geometry_msgs cv_bridge message_filters image_geometry pcl_ros nav_msgs tf2 tf2_geometry_msgs tf2_ros nodelet
#  DEPENDS system_lib
)

###########
## Build ##
###########

# Build the sptam node
add_subdirectory( src/ros )

# When building standalone
else() # CATKIN_DEVEL_PREFIX

add_subdirectory( src/standAlone )

endif() # CATKIN_DEVEL_PREFIX
