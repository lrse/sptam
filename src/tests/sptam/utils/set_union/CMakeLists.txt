cmake_minimum_required(VERSION 2.6)
project( sptam )

include_directories(${SPTAM_INCLUDE_DIRS})

# Add test cpp file
add_executable(test_set_union test_set_union.cpp)
target_link_libraries(test_set_union gtest gtest_main)

add_test(
  NAME test_set_union
  COMMAND test_set_union
)
