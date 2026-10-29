#----------------------------------------------------------------
# Generated CMake target import file for configuration "Release".
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "rdl::rdl_core" for configuration "Release"
set_property(TARGET rdl::rdl_core APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(rdl::rdl_core PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/librdl_core.so"
  IMPORTED_SONAME_RELEASE "librdl_core.so"
  )

list(APPEND _cmake_import_check_targets rdl::rdl_core )
list(APPEND _cmake_import_check_files_for_rdl::rdl_core "${_IMPORT_PREFIX}/lib/librdl_core.so" )

# Import target "rdl::rdl_ik" for configuration "Release"
set_property(TARGET rdl::rdl_ik APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(rdl::rdl_ik PROPERTIES
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/librdl_ik.so"
  IMPORTED_SONAME_RELEASE "librdl_ik.so"
  )

list(APPEND _cmake_import_check_targets rdl::rdl_ik )
list(APPEND _cmake_import_check_files_for_rdl::rdl_ik "${_IMPORT_PREFIX}/lib/librdl_ik.so" )

# Import target "rdl::rdl_calib" for configuration "Release"
set_property(TARGET rdl::rdl_calib APPEND PROPERTY IMPORTED_CONFIGURATIONS RELEASE)
set_target_properties(rdl::rdl_calib PROPERTIES
  IMPORTED_LINK_DEPENDENT_LIBRARIES_RELEASE "rdl::rdl_core"
  IMPORTED_LOCATION_RELEASE "${_IMPORT_PREFIX}/lib/librdl_calib.so"
  IMPORTED_SONAME_RELEASE "librdl_calib.so"
  )

list(APPEND _cmake_import_check_targets rdl::rdl_calib )
list(APPEND _cmake_import_check_files_for_rdl::rdl_calib "${_IMPORT_PREFIX}/lib/librdl_calib.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
