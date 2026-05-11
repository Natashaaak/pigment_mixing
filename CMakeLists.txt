cmake_minimum_required(VERSION 3.20)

# Set vcpkg (for raytracing)
set(VCPKG_TARGET_TRIPLET "x64-linux" CACHE STRING "")
set(CMAKE_TOOLCHAIN_FILE "/opt/vcpkg/scripts/buildsystems/vcpkg.cmake" CACHE STRING "")

project(Pigment_Mixing_Main)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING "Choose the type of build." FORCE)
endif()

set(CMAKE_INTERPROCEDURAL_OPTIMIZATION TRUE)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Set C++ standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Set path to vcpkg
list(APPEND CMAKE_PREFIX_PATH "${CMAKE_SOURCE_DIR}/raytracing/vcpkg_installed/x64-linux")

# Add projects
add_subdirectory(matter)
add_subdirectory(raytracing)
