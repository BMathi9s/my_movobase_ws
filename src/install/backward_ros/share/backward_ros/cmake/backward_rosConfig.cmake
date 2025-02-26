
####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was backward_rosConfig.cmake.in                            ########

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
set_and_check(backward_ros_INCLUDE_DIRS "/home/movo_base/movobase_ws/src/install/backward_ros/include")
if(WIN32)
set_and_check(backward_ros_LIBRARIES "/home/movo_base/movobase_ws/src/install/backward_ros/lib/backward.lib")
else()
set_and_check(backward_ros_LIBRARIES "/home/movo_base/movobase_ws/src/install/backward_ros/lib/libbackward.so")
endif()
check_required_components(backward_ros)
include(/home/movo_base/movobase_ws/src/install/backward_ros/share/backward_ros/cmake/BackwardConfigAment.cmake)
