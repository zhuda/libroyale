cmake_minimum_required(VERSION 2.8)

set(CMAKE_PREFIX_PATH "../../share")

find_package(royale REQUIRED)

link_directories(${royale_LIB_DIR})

project (sample1)

add_executable(sample1
   sample1.cpp
   )
   
if (WIN32)
    add_custom_command(
        TARGET sample1 
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy "${royale_INSTALL_PATH}/bin/royale.dll"  $<TARGET_FILE_DIR:sample1>)
endif (WIN32)

if (APPLE)
    add_custom_command(
        TARGET sample1 
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy "${royale_INSTALL_PATH}/bin/libroyale.dylib"  $<TARGET_FILE_DIR:sample1>)
        
    add_custom_command(
        TARGET sample1 
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy "${royale_INSTALL_PATH}/bin/libroyale.${royale_VERSION}.dylib"  $<TARGET_FILE_DIR:sample1>)
endif (APPLE)

target_link_libraries(sample1 "${royale_LIBS}")
