#----------------------------------------------------------------
# Generated CMake target import file for configuration "Debug".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "rdl::rdl_core" for configuration "Debug"
set_property(TARGET rdl::rdl_core APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(rdl::rdl_core PROPERTIES
  IMPORTED_IMPLIB_DEBUG "${_IMPORT_PREFIX}/lib/rdl_core-d.lib"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/bin/rdl_core-d.dll"
  )

list(APPEND _cmake_import_check_targets rdl::rdl_core )
list(APPEND _cmake_import_check_files_for_rdl::rdl_core "${_IMPORT_PREFIX}/lib/rdl_core-d.lib" "${_IMPORT_PREFIX}/bin/rdl_core-d.dll" )

# Import target "rdl::rdl_ik" for configuration "Debug"
set_property(TARGET rdl::rdl_ik APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(rdl::rdl_ik PROPERTIES
  IMPORTED_IMPLIB_DEBUG "${_IMPORT_PREFIX}/lib/rdl_ik-d.lib"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/bin/rdl_ik-d.dll"
  )

list(APPEND _cmake_import_check_targets rdl::rdl_ik )
list(APPEND _cmake_import_check_files_for_rdl::rdl_ik "${_IMPORT_PREFIX}/lib/rdl_ik-d.lib" "${_IMPORT_PREFIX}/bin/rdl_ik-d.dll" )

# Import target "rdl::rdl_calib" for configuration "Debug"
set_property(TARGET rdl::rdl_calib APPEND PROPERTY IMPORTED_CONFIGURATIONS DEBUG)
set_target_properties(rdl::rdl_calib PROPERTIES
  IMPORTED_IMPLIB_DEBUG "${_IMPORT_PREFIX}/lib/rdl_calib-d.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_DEBUG "rdl::rdl_core"
  IMPORTED_LOCATION_DEBUG "${_IMPORT_PREFIX}/bin/rdl_calib-d.dll"
  )

list(APPEND _cmake_import_check_targets rdl::rdl_calib )
list(APPEND _cmake_import_check_files_for_rdl::rdl_calib "${_IMPORT_PREFIX}/lib/rdl_calib-d.lib" "${_IMPORT_PREFIX}/bin/rdl_calib-d.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
