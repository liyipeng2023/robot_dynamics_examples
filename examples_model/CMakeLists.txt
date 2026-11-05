message(STATUS "[RDL] Configuring support for examples_model...")

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

file(GLOB URDF_EXAMPLE_SOURCES "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp")

set(RDL_URDF_EXAMPLE_EXECUTABLES "")

foreach(src_file ${URDF_EXAMPLE_SOURCES})
    get_filename_component(example_name ${src_file} NAME_WE)
    add_executable(${example_name} ${src_file})
    target_include_directories(${example_name} PRIVATE ${EIGEN3_INCLUDE_DIR})
    target_link_libraries(${example_name} PRIVATE rdl::rdl_core)
    file(READ ${src_file} file_content)
    if(file_content MATCHES "robot_loader.h")
        if(TARGET rdl_model_loader)
            target_link_libraries(${example_name} PRIVATE rdl_model_loader)
            message(STATUS "[Auto-Detect Model] '${example_name}' depends on rdl_model_loader")
        endif()
    endif()


    if(file_content MATCHES "dynamics_calibration.h" OR file_content MATCHES "calibration")
        if(TARGET rdl::rdl_calib)
            target_link_libraries(${example_name} PRIVATE rdl::rdl_calib)
        endif()
    endif()

    if(file_content MATCHES "inverse_kinematics.h" OR file_content MATCHES "inverse_kinematics")
        if(TARGET rdl::rdl_ik)
            target_link_libraries(${example_name} PRIVATE rdl::rdl_ik)
        endif()
    endif()

    list(APPEND RDL_URDF_EXAMPLE_EXECUTABLES ${example_name})
endforeach()

set(RDL_URDF_EXAMPLE_EXECUTABLES ${RDL_URDF_EXAMPLE_EXECUTABLES} PARENT_SCOPE)

if(WIN32)
    foreach(test_target ${RDL_URDF_EXAMPLE_EXECUTABLES})
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
endif()