
###############################################
# declare program options with default values #
###############################################

SET(SHOW_POINT_CLOUD OFF CACHE BOOL "Enable/Disable PCL Point Cloud visualization for the standAlone.")
SET(SHOW_TRACKED_FRAMES OFF CACHE BOOL "Enable/Disable OpenCV frame visualization for the tracker.")
SET(SHOW_PROFILING ON CACHE BOOL "Enable/Disable Profiling of each step.")
SET(USE_LOOPCLOSURE OFF CACHE BOOL "Enable/Disable Loopclosure feature.")
SET(PARALLELIZE ON CACHE BOOL "Enable/Disable parallelization using IntelTBB.")
SET(OPENCV_THREADS "-1" CACHE STRING "Number of internal OpenCV threads (0: no paralellization, -1 leave as default, other number = number of threads)")
SET(ENABLE_FRIF OFF CACHE BOOL "Enable compilation of FRIF descriptor")

# CACHE is not enough for CMAKE_BUILD_TYPE, FORCE option must be applied
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Build type (Debug Release RelWithDebInfo MinSizeRel). RelWithDebInfo as default" FORCE)
endif(NOT CMAKE_BUILD_TYPE)

MESSAGE("Selected BUILD_TYPE: ${CMAKE_BUILD_TYPE}")

# Set flags
OPTION(SINGLE_THREAD "Run sptam as a single thread application in sequential fashion." OFF)

######################
# Set compiler flags #
######################

## Enable most warnings
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

## Disable annoying Eigen warnings
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")

## set EIGEN_DONT_VECTORIZE flag
#add_definitions(-DEIGEN_DONT_VECTORIZE)

## Enable C++11 support
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DCMAKE_CXX_STANDARD=14")

##########################
# set optimization flags #
##########################

set(OPTIMIZATION_FLAGS "-O4 -march=native")

# Flags for ARM (particularly tuned for Odroid-XU4)
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm")
  add_definitions(-DEIGEN_DONT_VECTORIZE) # disabling Eigen vectorization allows the following flags (vectorization)
  set(OPTIMIZATION_FLAGS "${OPTIMIZATION_FLAGS} -mcpu=cortex-a15.cortex-a7 -mtune=cortex-a15.cortex-a7 -march=native -mfpu=neon-vfpv4 -mfloat-abi=hard -funsafe-math-optimizations")
  # NOTE: unsafe-math is required in order for NEON vectorization instructions to be actually generated, since NEON is not IEEE754 compliant. this of course implies possible imprecision
  # in math operations. we tolerate this on ARM since we need it to run fast.
endif()

SET(ENABLE_TRACE OFF CACHE BOOL "Enable options to enable performance monitoring")

if(ENABLE_TRACE)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-omit-frame-pointer")
endif()

set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")
set(CMAKE_CXX_FLAGS_RELEASE "${OPTIMIZATION_FLAGS}")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${OPTIMIZATION_FLAGS} -g")

#################
# Check for TBB #
#################

# Find TBB
if (PARALLELIZE)
  find_package(TBB QUIET)
  if (TBB_FOUND)
    message("-- Enabling parallel code")
    add_definitions(-DENABLE_PARALLEL_CODE)
    include_directories(${TBB_INCLUDE_DIRS})
  else()
    message("-- Intel TBB library not found. WARNING: parallelized code will run serially!")
  endif()
endif()

###################################
# Forward CMake flags to compiler #
###################################

if( SHOW_TRACKED_FRAMES )
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DSHOW_TRACKED_FRAMES")
endif()

if( SHOW_PROFILING )
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DSHOW_PROFILING")
endif()

if( USE_ODOMETRY )
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_ODOMETRY")
endif()

if( USE_LOOPCLOSURE )
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DUSE_LOOPCLOSURE")
endif()

if( DISABLE_LOCALMAPPING )
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DDISABLE_LOCALMAPPING")
endif()

if( SINGLE_THREAD )
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DSINGLE_THREAD")
endif()

if (DEFINED OPENCV_THREADS)
  message("-- Setting OpenCV threads to: ${OPENCV_THREADS}")
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DOPENCV_THREADS=${OPENCV_THREADS}")
endif()

if (ENABLE_FRIF)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DENABLE_FRIF")
endif()
