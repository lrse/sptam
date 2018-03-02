cmake_minimum_required(VERSION 2.6)
project( sptam )

find_package(Boost COMPONENTS system REQUIRED)

include_directories(${SPTAM_INCLUDE_DIRS})

# Add test cpp file
add_executable(test_CovisibilityGraph test_CovisibilityGraph.cpp ${SPTAM_SOURCE_DIRS}/utils/CovisibilityGraph.cpp)

# Link test executable against gtest & gtest_main
target_link_libraries(test_CovisibilityGraph gtest gtest_main ${Boost_LIBRARIES})

add_test(
  NAME test_CovisibilityGraph
  COMMAND test_CovisibilityGraph
)
