cmake_minimum_required(VERSION 3.0.2)
project( sptam )

add_library(sptam_ros stereo_driver.cpp base_driver.cpp utils/tf_utils.cpp utils/opencv_parsers.cpp)

add_library(sptam_nodelet sptam_nodelet.cpp)
target_link_libraries(sptam_nodelet sptam_ros ${catkin_LIBRARIES} sptam)

add_executable(sptam_node sptam_node.cpp)
target_link_libraries(sptam_node sptam_ros ${catkin_LIBRARIES} sptam)
