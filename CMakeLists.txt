cmake_minimum_required(VERSION 3.12 FATAL_ERROR)

if(COMMAND CMAKE_POLICY)
  CMAKE_POLICY(SET CMP0003 NEW)
  CMAKE_POLICY(SET CMP0004 NEW)
endif(COMMAND CMAKE_POLICY)

project (sdformat14 VERSION 14.5.0)

# The protocol version has nothing to do with the package version.
# It represents the current version of SDFormat implemented by the software
set (SDF_PROTOCOL_VERSION 1.11)

OPTION(SDFORMAT_DISABLE_CONSOLE_LOGFILE "Disable the sdformat console logfile" OFF)

# BUILD_SDF is preserved for backwards compatibility but can be removed on the main branch
set (BUILD_SDF ON CACHE INTERNAL "Build SDF" FORCE)

#################################################
# Find gz-cmake
find_package(gz-cmake3 REQUIRED)
set(GZ_CMAKE_VER ${gz-cmake3_VERSION_MAJOR})

########################################
option(SKIP_PYBIND11
      "Skip generating Python bindings via pybind11"
      OFF)

# Python interfaces vars
include(CMakeDependentOption)
cmake_dependent_option(USE_SYSTEM_PATHS_FOR_PYTHON_INSTALLATION
      "Install python modules in standard system paths in the system"
      OFF "NOT SKIP_PYBIND11" OFF)

cmake_dependent_option(USE_DIST_PACKAGES_FOR_PYTHON
      "Use dist-packages instead of site-package to install python modules"
      OFF "NOT SKIP_PYBIND11" OFF)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (BUILD_SDF)
  gz_configure_project(
    NO_PROJECT_PREFIX
    REPLACE_INCLUDE_PATH sdf
    VERSION_SUFFIX)

  #################################################
  # Find tinyxml2.
  gz_find_package(TINYXML2 REQUIRED)

  ################################################
  # Find urdfdom parser. Logic:
  #
  #  1. if USE_INTERNAL_URDF is unset, try to use system installation, fallback to internal copy
  #  2. if USE_INTERNAL_URDF is set to True, use the internal copy
  #  3. if USE_INTERNAL_URDF is set to False, force to search system installation, fail on error
  if (NOT DEFINED USE_INTERNAL_URDF OR NOT USE_INTERNAL_URDF)
    gz_find_package(GzURDFDOM VERSION 1.0 QUIET)
    if (NOT GzURDFDOM_FOUND)
      if (NOT DEFINED USE_INTERNAL_URDF)
        # fallback to internal urdf
        set(USE_INTERNAL_URDF ON)
      else()
        gz_build_error("Couldn't find the urdfdom >= 1.0 system installation")
      endif()
    endif()
  endif()

  if (USE_INTERNAL_URDF)
    message(STATUS "Using internal URDF")
  endif()

  #################################################
  # Find gz command line utility:
  find_program(GZ_PROGRAM gz)
  # Note that CLI files are installed regardless of whether the dependency is
  # available during build time
  set(GZ_TOOLS_VER 2)

  #################################################
  # Find python
  if (SKIP_PYBIND11)
    message(STATUS "SKIP_PYBIND11 set - disabling python bindings")
    find_package(Python3 REQUIRED COMPONENTS Interpreter)
  else()
    find_package(Python3 REQUIRED
      COMPONENTS Interpreter
      OPTIONAL_COMPONENTS Development
    )
    if (NOT Python3_Development_FOUND)
      GZ_BUILD_WARNING("Python development libraries are missing: Python interfaces are disabled.")
    else()
      set(PYBIND11_PYTHON_VERSION 3)
      find_package(pybind11 2.4 CONFIG QUIET)

      if (pybind11_FOUND)
        message (STATUS "Searching for pybind11 - found version ${pybind11_VERSION}.")
      else()
        GZ_BUILD_WARNING("pybind11 is missing: Python interfaces are disabled.")
        message (STATUS "Searching for pybind11 - not found.")
      endif()
    endif()
  endif()

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
      execute_process(COMMAND "${Python3_EXECUTABLE}" "-c" "import re, ${module}; print(re.compile('/__init__.py.*').sub('',${module}.__file__))"
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
    gz_build_warning("Python psutil package not found. Memory leak tests will be skipped")
  endif()

  ########################################
  # Find gz math
  # Set a variable for generating ProjectConfig.cmake
  gz_find_package(gz-math7 VERSION REQUIRED)
  set(GZ_MATH_VER ${gz-math7_VERSION_MAJOR})

  ########################################
  # Find gz utils
  gz_find_package(gz-utils2 REQUIRED COMPONENTS cli)
  set(GZ_UTILS_VER ${gz-utils2_VERSION_MAJOR})

  gz_configure_build(HIDE_SYMBOLS_BY_DEFAULT QUIT_IF_BUILD_ERRORS)

  gz_create_packages()

  add_subdirectory(sdf)
  add_subdirectory(conf)
  add_subdirectory(doc)
  if (pybind11_FOUND AND NOT SKIP_PYBIND11)
    add_subdirectory(python)
  endif()
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
