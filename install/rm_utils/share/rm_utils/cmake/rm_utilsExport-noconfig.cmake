#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "rm_utils::rm_utils" for configuration ""
set_property(TARGET rm_utils::rm_utils APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(rm_utils::rm_utils PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/librm_utils.so"
  IMPORTED_SONAME_NOCONFIG "librm_utils.so"
  )

list(APPEND _cmake_import_check_targets rm_utils::rm_utils )
list(APPEND _cmake_import_check_files_for_rm_utils::rm_utils "${_IMPORT_PREFIX}/lib/librm_utils.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
