
####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was rdl-config.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

# 包含导出的 Targets 文件 (这里会自动生成 rdl::rdl_core, rdl::rdl_ik, rdl::rdl_calib 目标)
include("${CMAKE_CURRENT_LIST_DIR}/rdl-targets.cmake")

# 检查组件
check_required_components(rdl_core rdl_ik rdl_calib)

# 为旧版本或者不需要细分导入的客户端提供兼容的别名目标 rdl (让 rdl 默认指向 rdl_core)
if(NOT TARGET rdl)
    add_library(rdl INTERFACE IMPORTED)
    set_target_properties(rdl PROPERTIES
        INTERFACE_LINK_LIBRARIES "rdl::rdl_core;rdl::rdl_ik;rdl::rdl_calib"
    )
endif()

# 辅助宏（处理 RPATH 与 DLL 拷贝）
macro(MyRobotLib_setup_runtime _target)
    if(WIN32)
        # Windows: 只有 3.21+ 支持 $<TARGET_RUNTIME_DLLS> 自动拷贝动态库
        if(${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.21.0")
            add_custom_command(TARGET ${_target} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    $<TARGET_RUNTIME_DLLS:${_target}>
                    $<TARGET_FILE_DIR:${_target}>
                COMMAND_EXPAND_LISTS
                COMMENT "Windows: Auto-copying all runtime DLLs for ${_target}..."
            )
        else()
            # 兼容旧版本：手动拷贝核心 DLL 库
            add_custom_command(TARGET ${_target} POST_BUILD
                COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "$$<TARGET_FILE:rdl::rdl_core>"
                    "$$<TARGET_FILE:rdl::rdl_ik>"
                    "$$<TARGET_FILE:rdl::rdl_calib>"
                    "$<TARGET_FILE_DIR:${_target}>"
                COMMENT "Windows (Old CMake): Copying rdl DLLs..."
            )
        endif()
    else()
        # Unix/Linux/macOS: 处理 RPATH
        set_target_properties(${_target} PROPERTIES
            SKIP_BUILD_RPATH  FALSE
            BUILD_WITH_INSTALL_RPATH FALSE
            INSTALL_RPATH_USE_LINK_PATH TRUE
        )
        if(APPLE)
            set_target_properties(${_target} PROPERTIES MACOSX_RPATH TRUE)
        endif()
    endif()
endmacro()
