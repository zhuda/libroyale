cmake_minimum_required(VERSION 2.8)

set(CMAKE_PREFIX_PATH "../../share")

find_package(royale REQUIRED)

link_directories(${royale_LIB_DIR})

project (sample2)

add_executable(sample2
   sample2.cpp
   )
   
if (WIN32)
    add_custom_command(
        TARGET sample2 
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy "${royale_INSTALL_PATH}/bin/royale.dll"  $<TARGET_FILE_DIR:sample2>)
endif (WIN32)

if (APPLE)
    add_custom_command(
        TARGET sample2 
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy "${royale_INSTALL_PATH}/bin/libroyale.dylib"  $<TARGET_FILE_DIR:sample2>)
        
    add_custom_command(
        TARGET sample2 
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy "${royale_INSTALL_PATH}/bin/libroyale.${royale_VERSION}.dylib"  $<TARGET_FILE_DIR:sample2>)
endif (APPLE)

target_link_libraries(sample2 "${royale_LIBS}")
