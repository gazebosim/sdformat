cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

if(COMMAND CMAKE_POLICY)
  CMAKE_POLICY(SET CMP0003 NEW)
  CMAKE_POLICY(SET CMP0004 NEW)
endif(COMMAND CMAKE_POLICY)

project (sdformat10 VERSION 10.6.0)

# The protocol version has nothing to do with the package version.
# It represents the current version of SDFormat implemented by the software
set (SDF_PROTOCOL_VERSION 1.7)

OPTION(SDFORMAT_DISABLE_CONSOLE_LOGFILE "Disable the sdformat console logfile" OFF)

set (BUILD_SDF ON CACHE INTERNAL "Build SDF" FORCE)

#################################################
# Find ign-cmake
find_package(ignition-cmake2 2.10 REQUIRED)
set(IGN_CMAKE_VER ${ignition-cmake2_VERSION_MAJOR})

if (BUILD_SDF)
  ign_configure_project(
    NO_IGNITION_PREFIX
    REPLACE_IGNITION_INCLUDE_PATH sdf)

  message (STATUS "\n\n====== Finding 3rd Party Packages ======")
  message(STATUS "Building for arch: ${ARCH}")

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
  ign_find_package(ignition-tools)

  ################################################
  # Find the Python interpreter for running the
  # check_test_ran.py script
  find_package(PythonInterp 3 QUIET)

  ################################################
  # Find psutil python package for memory tests
  include (${PROJECT_SOURCE_DIR}/cmake/SDFUtils.cmake)
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
  ign_find_package(ignition-math6 VERSION REQUIRED)
  set(IGN_MATH_VER ${ignition-math6_VERSION_MAJOR})

  message (STATUS "----------------------------------------\n")

  ign_configure_build(QUIT_IF_BUILD_ERRORS)
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
