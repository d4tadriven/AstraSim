cmake_minimum_required(VERSION 3.14)
project(AstraSim VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Configure output directories
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

include(FetchContent)

# Eigen3 for high-performance math
FetchContent_Declare(
  Eigen3
  GIT_REPOSITORY https://gitlab.com/libeigen/eigen.git
  GIT_TAG 3.4.0
)
FetchContent_MakeAvailable(Eigen3)

# Catch2 for testing
FetchContent_Declare(
  Catch2
  GIT_REPOSITORY https://github.com/catchorg/Catch2.git
  GIT_TAG v3.4.0
)
FetchContent_MakeAvailable(Catch2)

# pybind11 for python bindings
FetchContent_Declare(
  pybind11
  GIT_REPOSITORY https://github.com/pybind/pybind11.git
  GIT_TAG v2.11.1
)
FetchContent_MakeAvailable(pybind11)

# Core Library
add_library(astrasim_core
    src/physics/rigid_body.cpp
)
target_include_directories(astrasim_core PUBLIC 
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(astrasim_core PUBLIC Eigen3::Eigen)

# Tests
enable_testing()
add_executable(astrasim_tests
    tests/test_rigid_body.cpp
)
target_link_libraries(astrasim_tests PRIVATE astrasim_core Catch2::Catch2WithMain)
add_test(NAME AstraSimTests COMMAND astrasim_tests)

# Python bindings module
pybind11_add_module(core src/bindings/python_bindings.cpp)
target_link_libraries(core PRIVATE astrasim_core)
install(TARGETS core DESTINATION astrasim)
