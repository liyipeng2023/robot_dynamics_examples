find_package(cppzmq CONFIG REQUIRED)
if (cppzmq_FOUND)
    message(STATUS "cppzmq is found.")
endif()

# 1. Compare URDF and MJCF loader using RDL
add_executable(rdl_robot_controller rdl_robot_controller.cpp)
target_include_directories(rdl_robot_controller PRIVATE ${EIGEN3_INCLUDE_DIR})
target_link_libraries(rdl_robot_controller PRIVATE
        rdl::rdl_core
        rdl_model_loader
        cppzmq
)

set(RDL_BENCHMARK_EXECUTABLES
        rdl_robot_controller
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