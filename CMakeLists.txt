cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

if(COMMAND CMAKE_POLICY)
  CMAKE_POLICY(SET CMP0003 NEW)
  CMAKE_POLICY(SET CMP0004 NEW)
endif(COMMAND CMAKE_POLICY)

project (sdformat12 VERSION 12.8.0)

# The protocol version has nothing to do with the package version.
# It represents the current version of SDFormat implemented by the software
set (SDF_PROTOCOL_VERSION 1.9)

OPTION(SDFORMAT_DISABLE_CONSOLE_LOGFILE "Disable the sdformat console logfile" OFF)

# BUILD_SDF is preserved for backwards compatibility but can be removed on the main branch
set (BUILD_SDF ON CACHE INTERNAL "Build SDF" FORCE)

#################################################
# Find ign-cmake
find_package(ignition-cmake2 2.10 REQUIRED)
set(IGN_CMAKE_VER ${ignition-cmake2_VERSION_MAJOR})

if (BUILD_SDF)
  ign_configure_project(
    NO_IGNITION_PREFIX
    REPLACE_IGNITION_INCLUDE_PATH sdf
    VERSION_SUFFIX)

  #################################################
  # Find tinyxml2.
  ign_find_package(TINYXML2 REQUIRED)

  ################################################
  # Find urdfdom parser. Logic:
  #
  #  1. if USE_INTERNAL_URDF is unset, try to use system installation, fallback to internal copy
  #  2. if USE_INTERNAL_URDF is set to True, use the internal copy
  #  3. if USE_INTERNAL_URDF is set to False, force to search system installation, fail on error
  if (NOT DEFINED USE_INTERNAL_URDF OR NOT USE_INTERNAL_URDF)
    ign_find_package(IgnURDFDOM VERSION 1.0 QUIET)
    if (NOT IgnURDFDOM_FOUND)
      if (NOT DEFINED USE_INTERNAL_URDF)
        # fallback to internal urdf
        set(USE_INTERNAL_URDF ON)
      else()
        ign_build_error("Couldn't find the urdfdom >= 1.0 system installation")
      endif()
    endif()
  endif()

  if (USE_INTERNAL_URDF)
    message(STATUS "Using internal URDF")
  endif()

  #################################################
  # Find ign command line utility:
  find_program(IGN_PROGRAM ign)
  # Note that CLI files are installed regardless of whether the dependency is
  # available during build time
  set(IGN_TOOLS_VER 1)

  #################################################
  # Copied from catkin/cmake/empy.cmake
  include(IgnPython)
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

  ################################################
  # Find psutil python package for memory tests
  find_python_module(psutil)
  if (NOT PY_PSUTIL)
    ign_build_warning("Python psutil package not found. Memory leak tests will be skipped")
  endif()

  ################################################
  # Find ruby executable to produce xml schemas
  find_program(RUBY ruby)
  if (NOT RUBY)
    ign_build_error ("Ruby version 1.9 is needed to build xml schemas")
  else()
    message(STATUS "Found ruby executable: ${RUBY}")
  endif()

  ########################################
  # Find ignition math
  # Set a variable for generating ProjectConfig.cmake
  ign_find_package(ignition-math6 VERSION 6.13 REQUIRED)
  set(IGN_MATH_VER ${ignition-math6_VERSION_MAJOR})

  ########################################
  # Find ignition utils
  ign_find_package(ignition-utils1 VERSION REQUIRED)
  set(IGN_UTILS_VER ${ignition-utils1_VERSION_MAJOR})

  ign_configure_build(HIDE_SYMBOLS_BY_DEFAULT QUIT_IF_BUILD_ERRORS)

  ign_create_packages()

  add_subdirectory(sdf)
  add_subdirectory(conf)
  add_subdirectory(doc)
endif(BUILD_SDF)

########################################
# Setup Codecheck

# Ignore vendored directories.
file(WRITE ${PROJECT_BINARY_DIR}/cppcheck.suppress
  "*:${PROJECT_SOURCE_DIR}/src/urdf/*\n"
  )

########################################
# Configure documentation uploader
configure_file("${CMAKE_SOURCE_DIR}/cmake/upload_doc.sh.in"
  ${CMAKE_BINARY_DIR}/upload_doc.sh @ONLY)

message(STATUS "Configuration successful. Type make to compile sdf")
