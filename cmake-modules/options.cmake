
###############################################
# declare program options with default values #
###############################################

option(SHOW_POINT_CLOUD     "PCL Point Cloud visualization (only for stand-alone version)" OFF)
option(SHOW_TRACKED_FRAMES  "OpenCV visualization of tracked frames." OFF)
option(SHOW_PROFILING       "Profiling of each step." ON)
option(USE_LOOPCLOSURE      "Use loopclosure." OFF)
option(PARALLELIZE          "Parallelization using IntelTBB." ON)
option(ENABLE_FRIF          "Enable FRIF descriptor" OFF)
option(SINGLE_THREAD        "Run tracker and mapper on a single thread in sequential fashion." OFF)
option(ENABLE_TRACE         "Enable options to enable performance monitoring" OFF)

set(OPENCV_THREADS "-1" CACHE STRING "Number of internal OpenCV threads (0: no paralellization, -1 leave as default, other number = number of threads)")

# CACHE is not enough for CMAKE_BUILD_TYPE, FORCE option must be applied
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING "Build type (Debug Release RelWithDebInfo MinSizeRel). RelWithDebInfo as default" FORCE)
endif()

######################
# Set compiler flags #
######################

## Enable most warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")

## Disable annoying Eigen warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations -Wno-int-in-bool-context")

## set EIGEN_DONT_VECTORIZE flag
#add_definitions(-DEIGEN_DONT_VECTORIZE)

## Require C++14 support, since we use specific features
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

##########################
# set optimization flags #
##########################

set(OPTIMIZATION_FLAGS "-O4 -march=native")

# Flags for ARM (particularly tuned for Odroid-XU4)
if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm")
  set(OPTIMIZATION_FLAGS "${OPTIMIZATION_FLAGS} -mcpu=cortex-a15.cortex-a7 -mtune=cortex-a15.cortex-a7 -march=native -mfpu=neon-vfpv4 -mfloat-abi=hard -funsafe-math-optimizations")
  # NOTE: unsafe-math is required in order for NEON vectorization instructions to be actually generated, since NEON is not IEEE754 compliant. this of course implies possible imprecision
  # in math operations. we tolerate this on ARM since we need it to run fast.
endif()

if( ENABLE_TRACE )
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fno-omit-frame-pointer")
endif()

set(CMAKE_CXX_FLAGS_DEBUG "-O0 -g")
set(CMAKE_CXX_FLAGS_RELEASE "${OPTIMIZATION_FLAGS}")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "${OPTIMIZATION_FLAGS} -g")

#################
# Check for TBB #
#################

# Find TBB
if( PARALLELIZE )
  find_package(TBB QUIET)
  if( TBB_FOUND )
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
  add_definitions(-DSHOW_TRACKED_FRAMES)
endif()

if( SHOW_PROFILING )
  add_definitions(-DSHOW_PROFILING)
endif()

if( SHOW_POINT_CLOUD )
  add_definitions(-DSHOW_POINT_CLOUD)
endif()

if( USE_ODOMETRY )
  add_definitions(-DUSE_ODOMETRY)
endif()

if( USE_LOOPCLOSURE )
  add_definitions(-DUSE_LOOPCLOSURE)
endif()

if( DISABLE_LOCALMAPPING )
  add_definitions(-DDISABLE_LOCALMAPPING)
endif()

if( SINGLE_THREAD )
  add_definitions(-DSINGLE_THREAD)
endif()

if(DEFINED OPENCV_THREADS)
  add_definitions(-DOPENCV_THREADS=${OPENCV_THREADS})
endif()

if( ENABLE_FRIF )
  add_definitions(-DENABLE_FRIF)
endif()

#######################
# Print option status #
#######################

if(NOT WIN32)
  string(ASCII 27 Esc)
  set(ColourReset "${Esc}[m")
  set(ColourBold  "${Esc}[1m")
  set(Red         "${Esc}[31m")
  set(Green       "${Esc}[32m")
  set(Yellow      "${Esc}[33m")
  set(Blue        "${Esc}[34m")
  set(Magenta     "${Esc}[35m")
  set(Cyan        "${Esc}[36m")
  set(White       "${Esc}[37m")
  set(BoldRed     "${Esc}[1;31m")
  set(BoldGreen   "${Esc}[1;32m")
  set(BoldYellow  "${Esc}[1;33m")
  set(BoldBlue    "${Esc}[1;34m")
  set(BoldMagenta "${Esc}[1;35m")
  set(BoldCyan    "${Esc}[1;36m")
  set(BoldWhite   "${Esc}[1;37m")
endif()

message(STATUS "Build type: " ${BoldGreen} ${CMAKE_BUILD_TYPE} ${ColourReset})
message(STATUS "Show point cloud: " ${BoldGreen}  ${SHOW_POINT_CLOUD} ${ColourReset})
message(STATUS "Show tracked frames: " ${BoldGreen}  ${SHOW_TRACKED_FRAMES} ${ColourReset})
message(STATUS "Show profiling: " ${BoldGreen}  ${SHOW_PROFILING} ${ColourReset})
message(STATUS "Use loopclosure: " ${BoldGreen}  ${USE_LOOPCLOSURE} ${ColourReset})
message(STATUS "Use TBB paralelization: " ${BoldGreen}  ${PARALLELIZE} ${ColourReset})
message(STATUS "Enable Frif: " ${BoldGreen} ${ENABLE_FRIF} ${ColourReset})
message(STATUS "Single thread mode: " ${BoldGreen} ${SINGLE_THREAD} ${ColourReset})
message(STATUS "Enable trace: " ${BoldGreen} ${ENABLE_TRACE} ${ColourReset})
message(STATUS "OpenCV threads: " ${BoldGreen} ${OPENCV_THREADS} ${ColourReset})
