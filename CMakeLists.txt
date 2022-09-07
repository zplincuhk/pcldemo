cmake_minimum_required(VERSION 2.6)
project(pcl_test)

find_package(PCL 1.9 REQUIRED)

include_directories(include())
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(pcl_test src/pcl_test.cpp)

target_link_libraries (pcl_test ${PCL_LIBRARIES})

