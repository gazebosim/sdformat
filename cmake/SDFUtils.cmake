#################################################
macro (sdf_setup_windows)
  # Need for M_PI constant
  add_definitions(-D_USE_MATH_DEFINES)
  # And force linking to MSVC dynamic runtime
  set(CMAKE_C_FLAGS_DEBUG "/MDd ${CMAKE_C_FLAGS_DEBUG}")
  set(CMAKE_C_FLAGS_RELEASE "/MD ${CMAKE_C_FLAGS_RELEASE}")
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
