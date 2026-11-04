cmake_minimum_required(VERSION 3.10)
project(benchmark_pinocchio CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 显式开启最高级别编译优化
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

# 寻找 Pinocchio 依赖包
find_package(pinocchio REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${pinocchio_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

add_executable(benchmark_pinocchio benchmark_pinocchio.cpp)
target_link_libraries(benchmark_pinocchio PRIVATE pinocchio::pinocchio)