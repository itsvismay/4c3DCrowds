#version 3.11 or later of CMake or needed later for installing GoogleTest
# so let's require it now.
cmake_minimum_required(VERSION 3.11-3.18)

project(spatialHash)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)

add_library(spatialHash INTERFACE)

# PUBLIC needed to make both hello.h and hello library available elsewhere in project
target_include_directories(${PROJECT_NAME}
    PUBLIC INTERFACE ${PROJECT_SOURCE_DIR}/include)

# Tell compiler to use C++20 features. The code doesn't actually use any of them.
# target_compile_features(spatialHash PUBLIC cxx_std_20)