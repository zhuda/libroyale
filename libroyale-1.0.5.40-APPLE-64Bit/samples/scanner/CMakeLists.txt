cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project (scanner)

find_package(PCL 1.2 REQUIRED)
set(CMAKE_PREFIX_PATH "../../share")
find_package(royale REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS} ${royale_LIB_DIR})
add_definitions(${PCL_DEFINITIONS})


add_executable(scanner
   main.cpp
   )

target_link_libraries(scanner "${royale_LIBS}" ${PCL_LIBRARIES})

if (WIN32)
    add_custom_command(
        TARGET scanner
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy "${royale_INSTALL_PATH}/bin/royale.dll"  $<TARGET_FILE_DIR:scanner>)
endif (WIN32)

if (APPLE)
    add_custom_command(
        TARGET scanner
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy "${royale_INSTALL_PATH}/bin/libroyale.dylib"  $<TARGET_FILE_DIR:scanner>)

    add_custom_command(
        TARGET scanner
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy "${royale_INSTALL_PATH}/bin/libroyale.${royale_VERSION}.dylib"  $<TARGET_FILE_DIR:scanner>)
endif (APPLE)
