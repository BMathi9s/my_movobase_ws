#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "omni_wheel_controller::omni_wheel_controller" for configuration ""
set_property(TARGET omni_wheel_controller::omni_wheel_controller APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(omni_wheel_controller::omni_wheel_controller PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libomni_wheel_controller.so"
  IMPORTED_SONAME_NOCONFIG "libomni_wheel_controller.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS omni_wheel_controller::omni_wheel_controller )
list(APPEND _IMPORT_CHECK_FILES_FOR_omni_wheel_controller::omni_wheel_controller "${_IMPORT_PREFIX}/lib/libomni_wheel_controller.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
