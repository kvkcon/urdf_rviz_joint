# generated from ament/cmake/core/templates/nameConfig.cmake.in

# prevent multiple inclusion
if(_x1_urdf_CONFIG_INCLUDED)
  # ensure to keep the found flag the same
  if(NOT DEFINED x1_urdf_FOUND)
    # explicitly set it to FALSE, otherwise CMake will set it to TRUE
    set(x1_urdf_FOUND FALSE)
  elseif(NOT x1_urdf_FOUND)
    # use separate condition to avoid uninitialized variable warning
    set(x1_urdf_FOUND FALSE)
  endif()
  return()
endif()
set(_x1_urdf_CONFIG_INCLUDED TRUE)

# output package information
if(NOT x1_urdf_FIND_QUIETLY)
  message(STATUS "Found x1_urdf: 0.0.0 (${x1_urdf_DIR})")
endif()

# warn when using a deprecated package
if(NOT "" STREQUAL "")
  set(_msg "Package 'x1_urdf' is deprecated")
  # append custom deprecation text if available
  if(NOT "" STREQUAL "TRUE")
    set(_msg "${_msg} ()")
  endif()
  # optionally quiet the deprecation message
  if(NOT ${x1_urdf_DEPRECATED_QUIET})
    message(DEPRECATION "${_msg}")
  endif()
endif()

# flag package as ament-based to distinguish it after being find_package()-ed
set(x1_urdf_FOUND_AMENT_PACKAGE TRUE)

# include all config extra files
set(_extras "")
foreach(_extra ${_extras})
  include("${x1_urdf_DIR}/${_extra}")
endforeach()
