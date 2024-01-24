#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "ctrl_pkg::frame_transforms" for configuration ""
set_property(TARGET ctrl_pkg::frame_transforms APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(ctrl_pkg::frame_transforms PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libframe_transforms.so"
  IMPORTED_SONAME_NOCONFIG "libframe_transforms.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS ctrl_pkg::frame_transforms )
list(APPEND _IMPORT_CHECK_FILES_FOR_ctrl_pkg::frame_transforms "${_IMPORT_PREFIX}/lib/libframe_transforms.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
