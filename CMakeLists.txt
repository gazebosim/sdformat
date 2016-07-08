cmake_minimum_required(VERSION 2.8.6 FATAL_ERROR)

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

project (SDFormat)
string (TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)

# The protocol version has nothing to do with the package version set below.
# It represents the current version of sdformat implement by the software
set (SDF_PROTOCOL_VERSION 1.6)

set (SDF_MAJOR_VERSION 4)
set (SDF_MINOR_VERSION 1)
set (SDF_PATCH_VERSION 1)

set (SDF_VERSION ${SDF_MAJOR_VERSION}.${SDF_MINOR_VERSION})
set (SDF_VERSION_FULL ${SDF_MAJOR_VERSION}.${SDF_MINOR_VERSION}.${SDF_PATCH_VERSION})

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

set (USE_FULL_RPATH OFF CACHE BOOL "Set to true to enable full rpath")


if (USE_FULL_RPATH)
  # use, i.e. don't skip the full RPATH for the build tree
  set(CMAKE_SKIP_BUILD_RPATH  FALSE)
  
  # when building, don't use the install RPATH already
  # (but later on when installing)
  set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE) 
  
  set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR}")
  
  # add the automatically determined parts of the RPATH
  # which point to directories outside the build tree to the install RPATH
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
  
  # the RPATH to be used when installing, but only if its not a system directory
  list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR}" isSystemDir)
  if("${isSystemDir}" STREQUAL "-1")
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/${LIB_INSTALL_DIR}")
  endif("${isSystemDir}" STREQUAL "-1")
endif()

set (BUILD_SDF ON CACHE INTERNAL "Build SDF" FORCE)
set (build_errors "" CACHE INTERNAL "build errors" FORCE)
set (build_warnings "" CACHE INTERNAL "build warnings" FORCE)

set (sdf_cmake_dir ${PROJECT_SOURCE_DIR}/cmake CACHE PATH
     "Location of CMake scripts")

include (${sdf_cmake_dir}/SDFUtils.cmake)

message (STATUS "\n\n====== Finding 3rd Party Packages ======")
 # Use of tinyxml. System installation on UNIX. Internal copy on WIN
if (UNIX)
  message (STATUS "Using system tinyxml")
  set (USE_EXTERNAL_TINYXML True)
elseif(WIN32)
  message (STATUS "Using internal tinyxml code")
  set (USE_EXTERNAL_TINYXML False)
else()
  message (STATUS "Unknown platform")
  BUILD_ERROR("Unknown platform")
endif()
include (${sdf_cmake_dir}/SearchForStuff.cmake)
message (STATUS "----------------------------------------\n")

#####################################
# Set the default build type
if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE "RelWithDebInfo" CACHE STRING 
    "Choose the type of build, options are: Debug Release RelWithDebInfo Profile Check" FORCE)
endif (NOT CMAKE_BUILD_TYPE)
string(TOUPPER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE_UPPERCASE)

set (BUILD_TYPE_PROFILE FALSE)
set (BUILD_TYPE_RELEASE FALSE)
set (BUILD_TYPE_RELWITHDEBINFO FALSE)
set (BUILD_TYPE_DEBUG FALSE)

if ("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "PROFILE")
  set (BUILD_TYPE_PROFILE TRUE)
elseif ("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "RELEASE")
  set (BUILD_TYPE_RELEASE TRUE)
elseif ("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "RELWITHDEBINFO")
  set (BUILD_TYPE_RELWITHDEBINFO TRUE)
elseif ("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "DEBUG")
  set (BUILD_TYPE_DEBUG TRUE)
elseif ("${CMAKE_BUILD_TYPE_UPPERCASE}" STREQUAL "COVERAGE")
  include (${project_cmake_dir}/CodeCoverage.cmake)
  set (BUILD_TYPE_DEBUG TRUE)
  SETUP_TARGET_FOR_COVERAGE(coverage ctest coverage)
else()
  build_error("CMAKE_BUILD_TYPE ${CMAKE_BUILD_TYPE} unknown. Valid options are: Debug Release RelWithDebInfo Profile Check")
endif()

#####################################
# Handle CFlags
unset (CMAKE_C_FLAGS_ALL CACHE)
unset (CMAKE_CXX_FLAGS CACHE)

# USE_HOST_CFLAGS (default TRUE)
# Will check building host machine for proper cflags
if(NOT DEFINED USE_HOST_CFLAGS OR USE_HOST_CFLAGS)
  message(STATUS "Enable host CFlags")
  include (${project_cmake_dir}/HostCFlags.cmake)
endif()

# USE_UPSTREAM_CFLAGS (default TRUE)
# Will use predefined ignition developers cflags
if(NOT DEFINED USE_UPSTREAM_CFLAGS OR USE_UPSTREAM_CFLAGS)
  message(STATUS "Enable upstream CFlags")
  include(${project_cmake_dir}/DefaultCFlags.cmake)
endif()

# Check if warning options are avaliable for the compiler and return WARNING_CXX_FLAGS variable
if (MSVC)
  set(WARN_LEVEL "/W4")

  # Unable to be filtered flags (failing due to limitations in filter_valid_compiler_warnings)
  # Handling exceptions rightly
  set(UNFILTERED_FLAGS "/EHsc")
else()
  # Equivalent to Wall (according to man gcc) but removed the unknown pragams since we use
  # MSVC only pragmas all over the code
  list(APPEND WARN_LEVEL -Waddress -Warray-bounds -Wcomment -Wformat -Wnonnull)
  list(APPEND WARN_LEVEL -Wparentheses -Wreorder -Wreturn-type)
  list(APPEND WARN_LEVEL -Wsequence-point -Wsign-compare -Wstrict-aliasing)
  list(APPEND WARN_LEVEL -Wstrict-overflow=1 -Wswitch -Wtrigraphs -Wuninitialized)
  list(APPEND WARN_LEVEL -Wunused-function -Wunused-label -Wunused-value)
  list(APPEND WARN_LEVEL -Wunused-variable -Wvolatile-register-var)

  # Unable to be filtered flags (failing due to limitations in filter_valid_compiler_warnings)
  set(UNFILTERED_FLAGS "-Wc++11-compat")
endif()

filter_valid_compiler_warnings(${WARN_LEVEL} -Wextra -Wno-long-long
  -Wno-unused-value -Wno-unused-value -Wno-unused-value -Wno-unused-value
  -Wfloat-equal -Wshadow -Winit-self -Wswitch-default
  -Wmissing-include-dirs -pedantic -Wno-pragmas)
set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}${WARNING_CXX_FLAGS} ${UNFILTERED_FLAGS}")

#################################################
# OS Specific initialization
if (UNIX)
  sdf_setup_unix()
else (WIN32)
  sdf_setup_windows()
else (APPLE)
  sdf_setup_apple()
endif()

#################################################
# Print warnings and errors
if ( build_warnings )
  message(STATUS "BUILD WARNINGS")
  foreach (msg ${build_warnings})
    message(STATUS ${msg})
  endforeach ()
  message(STATUS "END BUILD WARNINGS\n")
endif (build_warnings)

########### Add uninstall target ###############
configure_file(
  "${CMAKE_CURRENT_SOURCE_DIR}/cmake/cmake_uninstall.cmake.in"
  "${CMAKE_CURRENT_BINARY_DIR}/cmake/cmake_uninstall.cmake"
  IMMEDIATE @ONLY)
add_custom_target(uninstall
  "${CMAKE_COMMAND}" -P "${CMAKE_CURRENT_BINARY_DIR}/cmake/cmake_uninstall.cmake")

if (build_errors)
  message(STATUS "BUILD ERRORS: These must be resolved before compiling.")
  foreach (msg ${build_errors})
    message(STATUS ${msg})
  endforeach ()
  message(STATUS "END BUILD ERRORS\n")
  message (FATAL_ERROR "Errors encountered in build. Please see the BUILD ERRORS above.")

else (buid_errors)

  ########################################
  # Write the config.h file
  configure_file (${sdf_cmake_dir}/sdf_config.h.in
    ${PROJECT_BINARY_DIR}/sdf/sdf_config.h)
  sdf_install_includes("" ${PROJECT_BINARY_DIR}/sdf/sdf_config.h)

  message (STATUS "C Flags:${CMAKE_CXX_FLAGS} ${CMAKE_CXX_FLAGS_${CMAKE_BUILD_TYPE}}")
  message (STATUS "Build Type: ${CMAKE_BUILD_TYPE}")
  message (STATUS "Install path: ${CMAKE_INSTALL_PREFIX}")
 
  if (BUILD_SDF)
    include_directories(include
      ${PROJECT_BINARY_DIR}
      ${PROJECT_BINARY_DIR}/include
      )
  
    link_directories(${PROJECT_BINARY_DIR}/src)

    add_subdirectory(test)
    add_subdirectory(src)
    add_subdirectory(include/sdf)
    add_subdirectory(sdf)
    add_subdirectory(doc)
  endif(BUILD_SDF)

  ########################################
  # Make the package config file
  configure_file(${CMAKE_SOURCE_DIR}/cmake/sdformat_pc.in
    ${CMAKE_CURRENT_BINARY_DIR}/cmake/sdformat.pc @ONLY)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/cmake/sdformat.pc DESTINATION
    ${LIB_INSTALL_DIR}/pkgconfig COMPONENT pkgconfig)

  ########################################
  # Configure documentation uploader
  configure_file("${CMAKE_SOURCE_DIR}/cmake/upload_doc.sh.in"
    ${CMAKE_BINARY_DIR}/upload_doc.sh @ONLY)

  ########################################
  # Make the cmake config files
  set(PKG_NAME ${PROJECT_NAME})
  set(PKG_LIBRARIES sdformat)
  set(cmake_conf_file "cmake/sdf_config.cmake")
  set(cmake_conf_version_file "cmake/SDFormatConfigVersion.cmake")
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/${cmake_conf_file}.in" "${CMAKE_CURRENT_BINARY_DIR}/SDFormatConfig.cmake" @ONLY)
  # Use write_basic_package_version_file to generate a ConfigVersion file that
  # allow users of gazebo to specify the API or version to depend on
  # TODO: keep this instruction until deprecate Ubuntu/Precise and update with
  # https://github.com/Kitware/CMake/blob/v2.8.8/Modules/CMakePackageConfigHelpers.cmake 
  include(WriteBasicConfigVersionFile)
  write_basic_config_version_file(
      ${CMAKE_CURRENT_BINARY_DIR}/${cmake_conf_version_file}
      VERSION "${SDF_VERSION_FULL}"
      COMPATIBILITY SameMajorVersion)
  install(FILES 
            ${CMAKE_CURRENT_BINARY_DIR}/SDFormatConfig.cmake
            ${CMAKE_CURRENT_BINARY_DIR}/${cmake_conf_version_file}
          DESTINATION 
	    ${LIB_INSTALL_DIR}/cmake/${PROJECT_NAME_LOWER}/ 
	  COMPONENT cmake)

  ########################################
  # Package Creation: 
  include (${sdf_cmake_dir}/sdf_cpack.cmake)
  set (CPACK_PACKAGE_VERSION "${SDF_VERSION_FULL}")
  set (CPACK_PACKAGE_VERSION_MAJOR "${SDF_MAJOR_VERSION}")
  set (CPACK_PACKAGE_VERSION_MINOR "${SDF_MINOR_VERSION}")
  set (CPACK_PACKAGE_VERSION_PATCH "${SDF_PATCH_VERSION}")

  if (CPACK_GENERATOR)
    message(STATUS "Found CPack generators: ${CPACK_GENERATOR}")

    configure_file("${sdf_cmake_dir}/cpack_options.cmake.in"
                   ${SDF_CPACK_CFG_FILE} @ONLY)
    set(CPACK_PROJECT_CONFIG_FILE ${SDF_CPACK_CFG_FILE})
    include (CPack)
  endif()

  message(STATUS "Configuration successful. Type make to compile sdf")
endif(build_errors)
