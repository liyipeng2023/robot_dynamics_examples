#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "rdl::rdl_core" for configuration "Release"
set_property(TARGET rdl::rdl_core APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(rdl::rdl_core PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/rdl_core.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/rdl_core.dll"
  )

list(APPEND _cmake_import_check_targets rdl::rdl_core )
list(APPEND _cmake_import_check_files_for_rdl::rdl_core "${_IMPORT_PREFIX}/lib/rdl_core.lib" "${_IMPORT_PREFIX}/bin/rdl_core.dll" )

# Import target "rdl::rdl_ik" for configuration "Release"
set_property(TARGET rdl::rdl_ik APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(rdl::rdl_ik PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/rdl_ik.lib"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/rdl_ik.dll"
  )

list(APPEND _cmake_import_check_targets rdl::rdl_ik )
list(APPEND _cmake_import_check_files_for_rdl::rdl_ik "${_IMPORT_PREFIX}/lib/rdl_ik.lib" "${_IMPORT_PREFIX}/bin/rdl_ik.dll" )

# Import target "rdl::rdl_calib" for configuration "Release"
set_property(TARGET rdl::rdl_calib APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(rdl::rdl_calib PROPERTIES
  IMPORTED_IMPLIB_RELEASE "${_IMPORT_PREFIX}/lib/rdl_calib.lib"
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "rdl::rdl_core"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/bin/rdl_calib.dll"
  )

list(APPEND _cmake_import_check_targets rdl::rdl_calib )
list(APPEND _cmake_import_check_files_for_rdl::rdl_calib "${_IMPORT_PREFIX}/lib/rdl_calib.lib" "${_IMPORT_PREFIX}/bin/rdl_calib.dll" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
