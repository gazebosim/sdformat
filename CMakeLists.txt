cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

if(COMMAND CMAKE_POLICY)
  CMAKE_POLICY(SET CMP0003 NEW)
  CMAKE_POLICY(SET CMP0004 NEW)
endif(COMMAND CMAKE_POLICY)

project (sdformat13 VERSION 13.0.0)

# The protocol version has nothing to do with the package version set below.
# It represents the current version of sdformat implement by the software
set (SDF_PROTOCOL_VERSION 1.9)

set (project_cmake_dir ${PROJECT_SOURCE_DIR}/cmake
  CACHE PATH "Location of CMake scripts")

message (STATUS "${PROJECT_NAME} version ${PROJECT_VERSION_FULL}")

OPTION(SDFORMAT_DISABLE_CONSOLE_LOGFILE "Disable the sdformat console logfile" OFF)

set (BUILD_SDF ON CACHE INTERNAL "Build SDF" FORCE)

set (sdf_cmake_dir ${PROJECT_SOURCE_DIR}/cmake CACHE PATH
     "Location of CMake scripts")

#################################################
# Find ign-cmake
find_package(ignition-cmake2 2.9 REQUIRED)
set(IGN_CMAKE_VER ${ignition-cmake2_VERSION_MAJOR})

#################################################
# OS Specific initialization
if (WIN32)
  sdf_setup_windows()
endif ()

if (BUILD_SDF)
  ign_configure_project(REPLACE_IGNITION_INCLUDE_PATH sdf NO_IGNITION_PREFIX)

  message (STATUS "\n\n====== Finding 3rd Party Packages ======")
  include (${sdf_cmake_dir}/SDFUtils.cmake)
  include (${sdf_cmake_dir}/SearchForStuff.cmake)
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
