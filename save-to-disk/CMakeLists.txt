#  minimum required cmake version: 3.1.0
cmake_minimum_required(VERSION 3.1.0)

project(RealsenseExamplesSaveToDisk)

# Save the command line compile commands in the build output
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=c11")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
endif()

find_package(realsense2)
if(NOT realsense2_FOUND)
    message(FATAL_ERROR "\n\n Intel RealSense SDK 2.0 is missing, please install it from https://github.com/IntelRealSense/librealsense/releases\n\n")
endif()

find_package(Boost REQUIRED COMPONENTS system thread filesystem)#chrono)

# save-to-disk
add_executable(rs-save-to-disk rs-save-to-disk.cpp)
target_link_libraries(rs-save-to-disk ${DEPENDENCIES} realsense2 ${Boost_LIBRARIES})
include_directories(rs-save-to-disk ../third-party/)
set_target_properties (rs-save-to-disk PROPERTIES
    FOLDER "Examples"
)

install(
    TARGETS

    rs-save-to-disk

    RUNTIME DESTINATION
    ${CMAKE_INSTALL_PREFIX}/bin
)
