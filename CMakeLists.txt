cmake_minimum_required(VERSION 3.15)
project(robot_dynamics_examples LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# detect the type of operating system
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    set(OS_PREFIX "linux")
elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    set(OS_PREFIX "windows")
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

# 1. add test_floating_base
add_executable(test_floating_base ${CMAKE_SOURCE_DIR}/examples/test_floating_base.cpp)
target_include_directories(test_floating_base PRIVATE
    ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(test_floating_base PRIVATE rdl::rdl_core)

# 2. add test_6DOF_dynamics.cpp
add_executable(test_6DOF_dynamics ${CMAKE_SOURCE_DIR}/examples/test_6DOF_dynamics.cpp)
target_include_directories(test_6DOF_dynamics PRIVATE
        ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(test_6DOF_dynamics PRIVATE rdl::rdl_core)

# 3. add test_3DOF_dynamics.cpp
add_executable(test_3DOF_dynamics ${CMAKE_SOURCE_DIR}/examples/test_3DOF_dynamics.cpp)
target_include_directories(test_3DOF_dynamics PRIVATE
        ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(test_3DOF_dynamics PRIVATE rdl::rdl_core)

# 4. add test_dual_floating_robots.cpp
add_executable(test_dual_floating_robots ${CMAKE_SOURCE_DIR}/examples/test_dual_floating_robots.cpp)
target_include_directories(test_dual_floating_robots PRIVATE
        ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(test_dual_floating_robots PRIVATE rdl::rdl_core)

# 5. add test_6dof_arm_performance.cpp
add_executable(test_6dof_arm_performance ${CMAKE_SOURCE_DIR}/examples/test_6dof_arm_performance.cpp)
target_include_directories(test_6dof_arm_performance PRIVATE
        ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(test_6dof_arm_performance PRIVATE rdl::rdl_core)

# 6. add test_floating_base_performance.cpp
add_executable(test_floating_base_performance ${CMAKE_SOURCE_DIR}/examples/test_floating_base_performance.cpp)
target_include_directories(test_floating_base_performance PRIVATE
        ${EIGEN3_INCLUDE_DIR}
)
target_link_libraries(test_floating_base_performance PRIVATE rdl::rdl_core)

# Ensure the rdl DLL is available next to the executable after build
if(WIN32)
    set(TEST_TARGETS
            test_floating_base
            test_6DOF_dynamics
            test_3DOF_dynamics
            test_dual_floating_robots
    )

    foreach(test_target ${TEST_TARGETS})
        add_custom_command(
                TARGET ${test_target} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_RUNTIME_DLLS:${test_target}>
                $<TARGET_FILE_DIR:${test_target}>
                COMMAND_EXPAND_LISTS
                COMMENT "Copying runtime DLLs for ${test_target}..."
        )
    endforeach()
elseif(UNIX AND NOT APPLE)
    set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
    set(CMAKE_INSTALL_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/rdl/rdl-${RDL_PLATFORM_NAME}/lib")
endif()