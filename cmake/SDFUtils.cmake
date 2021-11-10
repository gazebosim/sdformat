################################################################################
#APPEND_TO_CACHED_STRING(_string _cacheDesc [items...])
# Appends items to a cached list.
MACRO (APPEND_TO_CACHED_STRING _string _cacheDesc)
  FOREACH (newItem ${ARGN})
    SET (${_string} "${${_string}} ${newItem}" CACHE INTERNAL ${_cacheDesc} FORCE)
  ENDFOREACH (newItem ${ARGN})
  #STRING(STRIP ${${_string}} ${_string})
ENDMACRO (APPEND_TO_CACHED_STRING)

################################################################################
# APPEND_TO_CACHED_LIST (_list _cacheDesc [items...]
# Appends items to a cached list.
MACRO (APPEND_TO_CACHED_LIST _list _cacheDesc)
  SET (tempList ${${_list}})
  FOREACH (newItem ${ARGN})
    LIST (APPEND tempList ${newItem})
  ENDFOREACH (newItem ${newItem})
  SET (${_list} ${tempList} CACHE INTERNAL ${_cacheDesc} FORCE)
ENDMACRO(APPEND_TO_CACHED_LIST)

#################################################
# Macro to turn a list into a string (why doesn't CMake have this built-in?)
MACRO (LIST_TO_STRING _string _list)
    SET (${_string})
    FOREACH (_item ${_list})
      SET (${_string} "${${_string}} ${_item}")
    ENDFOREACH (_item)
    #STRING(STRIP ${${_string}} ${_string})
ENDMACRO (LIST_TO_STRING)

#################################################
macro (sdf_add_library _name)
  set(LIBS_DESTINATION ${PROJECT_BINARY_DIR}/src)
  add_library(${_name} ${ARGN})
  set_target_properties(${_name} PROPERTIES DEFINE_SYMBOL "BUILDING_SDFORMAT_SHARED")
  set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${LIBS_DESTINATION})
  if (MSVC)
    set_target_properties( ${_name} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY ${LIBS_DESTINATION})
    set_target_properties( ${_name} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${LIBS_DESTINATION})
    set_target_properties( ${_name} PROPERTIES ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${LIBS_DESTINATION})
  endif ()
endmacro ()

#################################################
macro (sdf_setup_windows)
  # Need for M_PI constant
  add_definitions(-D_USE_MATH_DEFINES)
  # And force linking to MSVC dynamic runtime
  set(CMAKE_C_FLAGS_DEBUG "/MDd ${CMAKE_C_FLAGS_DEBUG}")
  set(CMAKE_C_FLAGS_RELEASE "/MD ${CMAKE_C_FLAGS_RELEASE}")
endmacro()

#################################################
# Macro to setup supported compiler warnings
# Based on work of Florent Lamiraux, Thomas Moulard, JRL, CNRS/AIST.
include(CheckCXXCompilerFlag)

macro(filter_valid_compiler_warnings)
  foreach(flag ${ARGN})
    CHECK_CXX_COMPILER_FLAG(${flag} R${flag})
    if(${R${flag}})
      set(WARNING_CXX_FLAGS "${WARNING_CXX_FLAGS} ${flag}")
    endif()
  endforeach()
endmacro()

#################################################
# Copied from catkin/cmake/empy.cmake
function(find_python_module module)
  # cribbed from http://www.cmake.org/pipermail/cmake/2011-January/041666.html
  string(TOUPPER ${module} module_upper)
  if(NOT PY_${module_upper})
    if(ARGC GREATER 1 AND ARGV1 STREQUAL "REQUIRED")
      set(${module}_FIND_REQUIRED TRUE)
    endif()
    # A module's location is usually a directory, but for
    # binary modules
    # it's a .so file.
    execute_process(COMMAND "${PYTHON_EXECUTABLE}" "-c" "import re, ${module}; print(re.compile('/__init__.py.*').sub('',${module}.__file__))"
      RESULT_VARIABLE _${module}_status
      OUTPUT_VARIABLE _${module}_location
      ERROR_QUIET OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(NOT _${module}_status)
      set(PY_${module_upper} ${_${module}_location} CACHE STRING "Location of Python module ${module}")
    endif(NOT _${module}_status)
  endif(NOT PY_${module_upper})
  include(FindPackageHandleStandardArgs)
  find_package_handle_standard_args(PY_${module} DEFAULT_MSG PY_${module_upper})
endfunction(find_python_module)
