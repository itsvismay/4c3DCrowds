# version 3.11 or later of CMake needed later for installing GoogleTest
# so let's require it now.
cmake_minimum_required(VERSION 3.11-3.18)

project(
    VscodeCmakeHello
    VERSION 0.1
    DESCRIPTION "Built with VSCode")

add_subdirectory(../external/polyscope polyscope)
add_subdirectory(../external/libigl libigl)

add_subdirectory(CustomContainers)   # look in hello subdirectory for CMakeLists.txt to process
add_subdirectory(hello)   # look in hello subdirectory for CMakeLists.txt to process
add_subdirectory(apps)    # look in apps subdirectory for CMakeLists.txt to process
