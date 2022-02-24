# version 3.11 or later of CMake or needed later for installing GoogleTest
# so let's require it now.
cmake_minimum_required(VERSION 3.11-3.18)

project(main)
#set(Eigen3_DIR )

find_package(Eigen3 3.3 REQUIRED NO_MODULE)


add_executable(testParticles testParticles.cpp)
add_executable(rods 4c3D.cpp)
add_executable(testARAP testARAP.cpp)

# We need hello.h and the hello library
target_include_directories(testARAP PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/../../external/libigl/include")

target_link_libraries(testParticles PRIVATE hello polyscope Eigen3::Eigen)
target_link_libraries(rods PRIVATE hello polyscope Eigen3::Eigen)
target_link_libraries(testARAP hello polyscope Eigen3::Eigen)

# Tell C++ compiler to use C++20 features. We don't actually use any of them.
target_compile_features(testParticles PUBLIC cxx_std_20)
target_compile_features(rods PUBLIC cxx_std_20)
target_compile_features(testARAP PUBLIC cxx_std_20)