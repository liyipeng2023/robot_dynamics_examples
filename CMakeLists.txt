cmake_minimum_required(VERSION 3.15)
project(robot_dynamics_examples LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(WITH_EXAMPLES_MODEL "Build examples with support for loading model from files" OFF)

# detect the type of operating system
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(OS_PREFIX "linux")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    set(OS_PREFIX "windows")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Darwin")
    set(OS_PREFIX "macos")
else()
    message(FATAL_ERROR "Unsupported operating system: ${CMAKE_SYSTEM_NAME}")
endif()
# detect the cpu architecture
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|AMD64|amd64")
    set(ARCH_SUFFIX "x86_64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|ARM64|arm64")
    set(ARCH_SUFFIX "arm64")
else()
    message(FATAL_ERROR "Unsupported architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

set(RDL_PLATFORM_FOLDER "rdl-${OS_PREFIX}-${ARCH_SUFFIX}")
message(STATUS "Detected target platform folder: ${RDL_PLATFORM_FOLDER}")
set(rdl_DIR "${CMAKE_CURRENT_SOURCE_DIR}/rdl/${RDL_PLATFORM_FOLDER}/lib/cmake/rdl")
if(NOT EXISTS "${rdl_DIR}")
    message(FATAL_ERROR "rdl_DIR path does not exist: ${rdl_DIR}\nPlease verify your rdl folder structure.")
endif()
find_package(rdl REQUIRED)

# path of eigen3
set(EIGEN3_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/3rdparties/eigen3.4.1")
message(STATUS "Eigen include: ${EIGEN3_INCLUDE_DIR}")

# add examples
add_subdirectory(examples)

# add examples_model
if (WITH_EXAMPLES_MODEL)
    message(STATUS "Add examples with dynamics created from files (urdfdom and tinyxml2 are required).")
    add_subdirectory(examples_model)
endif ()