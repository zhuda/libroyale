cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(processing)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (processing main.cpp
filter.h
filter.cpp
resample.h
resample.cpp
pairwise.h
pairwise.cpp
constant.h
triangulation.h
Timer.h)
target_link_libraries (processing ${PCL_LIBRARIES})
