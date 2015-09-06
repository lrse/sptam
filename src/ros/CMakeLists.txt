cmake_minimum_required(VERSION 2.8.3)
project( sptam )

## Enable most warnings
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wno-deprecated-declarations -std=c++11")

## Enable C++11 support
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## maximize optimization
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O4")

# TODO check for SPTAM_LIBRARIES to be defined

add_executable( sptam_node main.cpp sptam_node.cpp )
target_link_libraries( sptam_node ${SPTAM_LIBRARIES} )
