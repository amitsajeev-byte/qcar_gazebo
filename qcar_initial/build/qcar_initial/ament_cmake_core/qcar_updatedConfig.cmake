# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_qcar_initial_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED qcar_initial_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(qcar_initial_FOUND FALSE)
  elseif(NOT qcar_initial_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(qcar_initial_FOUND FALSE)
  endif()
  return()
endif()
set(_qcar_initial_CONFIG_INCLUDED TRUE)

# output package information
if(NOT qcar_initial_FIND_QUIETLY)
  message(STATUS "Found qcar_initial: 0.0.0 (${qcar_initial_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'qcar_initial' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${qcar_initial_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(qcar_initial_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "ament_cmake_export_include_directories-extras.cmake;ament_cmake_export_libraries-extras.cmake;ament_cmake_export_dependencies-extras.cmake")
foreach(_extra ${_extras})
  include("${qcar_initial_DIR}/${_extra}")
endforeach()
