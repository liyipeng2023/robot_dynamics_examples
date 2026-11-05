find_package(Pinocchio REQUIRED)
find_package(urdfdom REQUIRED)
find_package(tinyxml2 REQUIRED)

if(NOT TARGET rdl_model_loader)
    add_library(rdl_model_loader STATIC
            ../utils/robot_loader.cpp
            ../utils/urdf_parser_impl.cpp
            ../utils/mjcf_parser_impl.cpp
    )
    target_include_directories(rdl_model_loader PRIVATE ${EIGEN3_INCLUDE_DIR})
    target_link_libraries(rdl_model_loader PUBLIC
            rdl::rdl_core
            urdfdom::urdfdom_model
            urdfdom::urdfdom_world
            tinyxml2::tinyxml2
    )

    target_include_directories(rdl_model_loader PUBLIC
            $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/utils>
    )
endif()

# 1. comparing the outputs of crba, aba, and rnea
add_executable(compare_rdl_pinocchio compare_rdl_pinocchio.cpp)
target_link_libraries(compare_rdl_pinocchio PRIVATE
        rdl::rdl_core
        rdl_model_loader
        pinocchio::pinocchio
)

# 2. Pinocchio performance test
add_library(eigen_opt_interface INTERFACE)
target_compile_options(eigen_opt_interface INTERFACE
        -O3
        -march=armv8-a+simd
        -flto
)
target_link_options(eigen_opt_interface INTERFACE
        -flto
)
target_compile_definitions(eigen_opt_interface INTERFACE
        EIGEN_NO_DEBUG
        EIGEN_DONT_PARALLELIZE
)
add_executable(benchmark_pinocchio benchmark_pinocchio.cpp)
target_link_libraries(benchmark_pinocchio PRIVATE eigen_opt_interface
        pinocchio::pinocchio
        eigen_opt_interface
)

# 3. RDL performance test
add_executable(benchmark_rdl benchmark_rdl.cpp)
target_include_directories(benchmark_rdl PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(benchmark_rdl PRIVATE
        rdl::rdl_core
        rdl_model_loader
)