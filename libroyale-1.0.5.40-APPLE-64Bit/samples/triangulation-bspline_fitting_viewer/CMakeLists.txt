cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(3dm_viewer)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (3dm_viewer 3dm_viewer.cpp)
target_link_libraries (3dm_viewer ${PCL_LIBRARIES})
