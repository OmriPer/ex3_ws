cmake_minimum_required(VERSION 3.16)
project(pipuck_ws CXX)

# C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Output dirs
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)

# Find ARGoS core and modules
find_package(ARGoS REQUIRED)
# Include ARGoS directories
include_directories(${ARGOS_INCLUDE_DIR})
# Link ARGoS libraries
link_directories(${ARGOS_LIBRARY_DIR})

add_subdirectory(controllers)
add_subdirectory(loop_functions)
