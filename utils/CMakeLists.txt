#add_library(rdl_utils INTERFACE)
#target_include_directories(rdl_utils INTERFACE
#        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#        ${EIGEN3_INCLUDE_DIR}
#)

if(WITH_EXAMPLES_MODEL OR WITH_BENCHMARK)
    find_package(urdfdom REQUIRED)
    find_package(tinyxml2 REQUIRED)

    if(NOT TARGET rdl_model_loader)
        add_library(rdl_model_loader STATIC
                ./src/robot_loader.cpp
                ./src/urdf_parser_impl.cpp
                ./src/mjcf_parser_impl.cpp
        )
        target_include_directories(rdl_model_loader PRIVATE ${EIGEN3_INCLUDE_DIR} ./include)
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
endif()
