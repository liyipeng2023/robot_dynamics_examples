file(GLOB EXAMPLE_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")
set(RDL_EXAMPLE_EXECUTABLES "")

foreach(src_file ${EXAMPLE_SOURCES})
    get_filename_component(example_name ${src_file} NAME_WE)
    add_executable(${example_name} ${src_file})
    target_link_libraries(${example_name} PRIVATE rdl_model_loader)
    target_include_directories(${example_name} PRIVATE ${EIGEN3_INCLUDE_DIR})
    # 读取 cpp 文件内容，静态分析其头文件包含情况
    file(READ ${src_file} file_content)

    # 1. 默认所有测试都必须链接核心库
    target_link_libraries(${example_name} PRIVATE rdl::rdl_core)

    # 2. 如果源码中包含了特定头文件，自动按需追加链接对应的库
    if(file_content MATCHES "dynamics_calibration.h" OR file_content MATCHES "kinematic_calibration.h" OR file_content MATCHES "calibration")
        if(TARGET rdl::rdl_calib)
            target_link_libraries(${example_name} PRIVATE rdl::rdl_calib)
            message(STATUS "[Auto-Detect] '${example_name}' depends on rdl_calib")
        endif()
    endif()

    if(file_content MATCHES "inverse_kinematics.h" OR file_content MATCHES "inverse_kinematics")
        if(TARGET rdl::rdl_ik)
            target_link_libraries(${example_name} PRIVATE rdl::rdl_ik)
            message(STATUS "[Auto-Detect] '${example_name}' depends on rdl_ik")
        endif()
    endif()

    list(APPEND RDL_EXAMPLE_EXECUTABLES ${example_name})
endforeach()

set(RDL_EXAMPLE_EXECUTABLES ${RDL_EXAMPLE_EXECUTABLES} PARENT_SCOPE)

# Ensure the rdl DLL is available next to the executable after build
if(WIN32)
    foreach(test_target ${RDL_EXAMPLE_EXECUTABLES})
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