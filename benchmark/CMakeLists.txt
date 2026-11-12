find_package(pinocchio REQUIRED)

# 1. comparing the outputs of crba, aba, and rnea
add_executable(compare_rdl_pinocchio compare_rdl_pinocchio.cpp)
target_include_directories(compare_rdl_pinocchio PRIVATE ${pinocchio_INCLUDE_DIRS} )
target_link_libraries(compare_rdl_pinocchio PRIVATE
        rdl::rdl_core
        rdl_model_loader
        urdfdom::urdfdom_model
        urdfdom::urdfdom_world
        pinocchio::pinocchio
)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_compile_options(compare_rdl_pinocchio PRIVATE $<$<CONFIG:Release>:-O3 -march=native -DNDEBUG>)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    target_compile_options(compare_rdl_pinocchio PRIVATE $<$<CONFIG:Release>:/O2 /arch:AVX2 /Gy /GL /DNDEBUG /DEIGEN_NO_DEBUG>)
else()
    target_compile_options(compare_rdl_pinocchio PRIVATE $<$<CONFIG:Release>:-O3 -mpu=native -DNDEBUG>)
endif()

# 2. Pinocchio performance test
add_executable(benchmark_pinocchio benchmark_pinocchio.cpp)
target_include_directories(benchmark_pinocchio PRIVATE ${pinocchio_INCLUDE_DIRS} )
target_link_libraries(benchmark_pinocchio PRIVATE
        pinocchio::pinocchio
)
if(CMAKE_SYSTEM_NAME STREQUAL "Linux")
    target_compile_options(benchmark_pinocchio PRIVATE $<$<CONFIG:Release>:-O3 -march=native -DNDEBUG>)
elseif(CMAKE_SYSTEM_NAME STREQUAL "Windows")
    target_compile_options(benchmark_pinocchio PRIVATE $<$<CONFIG:Release>:/O2 /arch:AVX2 /Gy /GL /DNDEBUG /DEIGEN_NO_DEBUG>)
else()
    target_compile_options(benchmark_pinocchio PRIVATE $<$<CONFIG:Release>:-O3 -mpu=native -DNDEBUG>)
endif()

# 3. RDL performance test
add_executable(benchmark_rdl benchmark_rdl.cpp)
target_include_directories(benchmark_rdl PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(benchmark_rdl PRIVATE
        rdl::rdl_core
        rdl_model_loader
)

set(RDL_BENCHMARK_EXECUTABLES
        compare_rdl_pinocchio
        benchmark_pinocchio
        benchmark_rdl
)

if(WIN32)
    foreach(test_target ${RDL_BENCHMARK_EXECUTABLES})
        add_custom_command(
                TARGET ${test_target} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_RUNTIME_DLLS:${test_target}>
                $<TARGET_FILE_DIR:${test_target}>
                COMMAND_EXPAND_LISTS
                COMMENT "Copying runtime DLLs for ${test_target}..."
        )
    endforeach()
elseif(UNIX)
    set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)
    if(APPLE)
        set(CMAKE_INSTALL_RPATH "@loader_path/../lib")
    else()
        set(CMAKE_INSTALL_RPATH "${CMAKE_CURRENT_SOURCE_DIR}/../rdl/lib")
    endif()

    foreach(test_target ${RDL_BENCHMARK_EXECUTABLES})
        set_target_properties(${test_target} PROPERTIES
                INSTALL_RPATH "${CMAKE_INSTALL_RPATH}"
        )
    endforeach()
endif()