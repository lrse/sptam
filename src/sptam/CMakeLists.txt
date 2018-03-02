cmake_minimum_required(VERSION 3.0.2)
project( sptam )

FIND_PACKAGE(SuiteSparse)
include_directories(${SUITESPARSE_INCLUDE_DIRS})

# List of files to compile
file(GLOB SPTAM_SRCS *.cpp utils/*.cpp)

# Compile the tracker visualization library
if( SHOW_TRACKED_FRAMES )
  set(SPTAM_SRCS ${SPTAM_SRCS} utils/draw/Draw.cpp)
endif()

# Compile the profiling library
if( SHOW_PROFILING )
  set(SPTAM_SRCS ${SPTAM_SRCS} utils/log/Logger.cpp)
endif()

# Build the sptam library
add_library(sptam SHARED ${SPTAM_SRCS} ${LC_SRCS})
target_link_libraries(sptam ${OpenCV_LIBRARIES} ${G2O_LIBRARIES} ${SUITESPARSE_LIBRARIES} ${CHOLMOD_LIBRARIES} cxsparse ${DLD_LIBRARIES} ${OPENGV_LIBS} ${YamlCpp_LIBRARIES} ${Boost_LIBRARIES} ${TBB_LIBRARIES})

set(SPTAM_SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)
set(SPTAM_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR} PARENT_SCOPE)
