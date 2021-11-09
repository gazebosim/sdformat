cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)

if(COMMAND CMAKE_POLICY)
  CMAKE_POLICY(SET CMP0003 NEW)
  CMAKE_POLICY(SET CMP0004 NEW)
endif(COMMAND CMAKE_POLICY)
enable_testing()

# with -fPIC
if(UNIX AND NOT WIN32)
  set (CMAKE_INSTALL_PREFIX "/usr" CACHE STRING "Install Prefix")
 find_program(CMAKE_UNAME uname /bin /usr/bin /usr/local/bin )
 if(CMAKE_UNAME)
   exec_program(uname ARGS -m OUTPUT_VARIABLE CMAKE_SYSTEM_PROCESSOR)
   set(CMAKE_SYSTEM_PROCESSOR ${CMAKE_SYSTEM_PROCESSOR} CACHE INTERNAL
     "processor type (i386 and x86_64)")
   if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
     ADD_DEFINITIONS(-fPIC)
   endif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
 endif(CMAKE_UNAME)
endif()

project (sdformat13 VERSION 13.0)

# The protocol version has nothing to do with the package version set below.
# It represents the current version of sdformat implement by the software
set (SDF_PROTOCOL_VERSION 1.9)

set (SDF_MAJOR_VERSION 13)
set (SDF_MINOR_VERSION 0)
set (SDF_PATCH_VERSION 0)

set (SDF_VERSION ${SDF_MAJOR_VERSION}.${SDF_MINOR_VERSION})
set (SDF_VERSION_FULL ${SDF_MAJOR_VERSION}.${SDF_MINOR_VERSION}.${SDF_PATCH_VERSION}~pre1)

string (TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)
string(REGEX REPLACE "[0-9]+" "" PROJECT_NAME_NO_VERSION ${PROJECT_NAME})
string(TOLOWER ${PROJECT_NAME_NO_VERSION} PROJECT_NAME_NO_VERSION_LOWER)
set(PROJECT_EXPORT_NAME ${PROJECT_NAME_LOWER})
set(PROJECT_LIBRARY_TARGET_NAME ${PROJECT_NAME_LOWER})

set (project_cmake_dir ${PROJECT_SOURCE_DIR}/cmake
  CACHE PATH "Location of CMake scripts")

message (STATUS "${PROJECT_NAME} version ${SDF_VERSION_FULL}")
set (CMAKE_INCLUDE_DIRECTORIES_PROJECT_BEFORE ON)

# Include GNUInstallDirs to get canonical paths
include(GNUInstallDirs)

# Default test type for test all over source
set(TEST_TYPE "UNIT")

# developer's option to cache PKG_CONFIG_PATH and
# LD_LIBRARY_PATH for local installs
if(PKG_CONFIG_PATH)
  set (ENV{PKG_CONFIG_PATH} ${PKG_CONFIG_PATH}:$ENV{PKG_CONFIG_PATH})
endif()
if(LD_LIBRARY_PATH)
  set (ENV{LD_LIBRARY_PATH} ${LD_LIBRARY_PATH}:$ENV{LD_LIBRARY_PATH})
endif()

set (INCLUDE_INSTALL_DIR "${CMAKE_INSTALL_INCLUDEDIR}/sdformat-${SDF_VERSION}/sdf")
set (LIB_INSTALL_DIR ${CMAKE_INSTALL_LIBDIR} CACHE STRING "Installation directory for libraries (relative to CMAKE_INSTALL_PREFIX)")
set (BIN_INSTALL_DIR ${CMAKE_INSTALL_BINDIR} CACHE STRING "Installation directory for binaries (relative to CMAKE_INSTALL_PREFIX)")

# set(PKG_NAME SDFormat)
# set(sdf_target sdformat${SDF_MAJOR_VERSION})
# set(sdf_config_install_dir "${LIB_INSTALL_DIR}/cmake/${PROJECT_NAME_LOWER}/")
# set(sdf_import_target_name ${PROJECT_EXPORT_NAME}::${sdf_target})
# set(sdf_target_output_filename "${sdf_target}-targets.cmake")

OPTION(SDFORMAT_DISABLE_CONSOLE_LOGFILE "Disable the sdformat console logfile" OFF)

set (BUILD_SDF ON CACHE INTERNAL "Build SDF" FORCE)
set (build_errors "" CACHE INTERNAL "build errors" FORCE)
set (build_warnings "" CACHE INTERNAL "build warnings" FORCE)

set (sdf_cmake_dir ${PROJECT_SOURCE_DIR}/cmake CACHE PATH
     "Location of CMake scripts")

message (STATUS "\n\n====== Finding 3rd Party Packages ======")
include (${sdf_cmake_dir}/SDFUtils.cmake)
include (${sdf_cmake_dir}/SearchForStuff.cmake)
message (STATUS "----------------------------------------\n")

#################################################
# OS Specific initialization
if (WIN32)
  sdf_setup_windows()
endif ()

if (BUILD_SDF)
  include_directories(include
    ${PROJECT_BINARY_DIR}
    ${PROJECT_BINARY_DIR}/include
    )

  link_directories(${PROJECT_BINARY_DIR}/src)

  if (NOT DEFINED BUILD_TESTING OR BUILD_TESTING)
    set(BUILD_SDF_TEST TRUE)
  else()
    set(BUILD_SDF_TEST FALSE)
  endif()

  ign_configure_project(NO_IGNITION_PREFIX)
  ign_configure_build(QUIT_IF_BUILD_ERRORS)
  ign_create_packages()
  ign_create_docs()

  # TODO: add header that redirects "sdf/sdf_config.h" to "${ignitin_prefix}/sdformat/config.hh"
  # install(
  #   FILES
  #     ${PROJECT_BINARY_DIR}/sdf/sdf_config.h
  #   DESTINATION
  #     ${IGN_INCLUDE_INSTALL_DIR_FULL}/sdf
  # )

  if (BUILD_SDF_TEST)
    # add_subdirectory(test)
  endif()
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
