include (FindPkgConfig)

# Detect the architecture
include (${project_cmake_dir}/TargetArch.cmake)
target_architecture(ARCH)
message(STATUS "Building for arch: ${ARCH}")

########################################
# Find Boost, if not specified manually
if (WIN32)
  set(Boost_USE_STATIC_LIBS       OFF)
  set(Boost_USE_MULTITHREADED      ON)
  set(Boost_USE_STATIC_RUNTIME    OFF)
endif()

include(FindBoost)
find_package(Boost ${MIN_BOOST_VERSION})

if (NOT Boost_FOUND)
  set (BUILD_SDF OFF CACHE INTERNAL "Build SDF" FORCE)
  BUILD_ERROR ("Boost not found. Please install system boost version ${MIN_BOOST_VERSION} or higher.")
endif()

if (USE_EXTERNAL_TINYXML)
  #################################################
  # Find tinyxml. Only debian distributions package tinyxml with a pkg-config
  # Use pkg_check_modules and fallback to manual detection (needed, at least, for MacOS)
  pkg_check_modules(tinyxml tinyxml)
  if (NOT tinyxml_FOUND)
    find_path (tinyxml_include_dirs tinyxml.h ${tinyxml_include_dirs} ENV CPATH)
    find_library(tinyxml_LIBRARIES NAMES tinyxml)
    set (tinyxml_FAIL False)
    if (NOT tinyxml_include_dirs)
      message (STATUS "Looking for tinyxml headers - not found")
      set (tinyxml_FAIL True)
    endif()
    if (NOT tinyxml_LIBRARIES)
      message (STATUS "Looking for tinyxml library - not found")
      set (tinyxml_FAIL True)
    endif()
  endif()

  if (tinyxml_FAIL)
    message (STATUS "Looking for tinyxml.h - not found")
    BUILD_ERROR("Missing: tinyxml")
  endif()
else()
  # Needed in WIN32 since in UNIX the flag is added in the code installed
  add_definitions(-DTIXML_USE_STL)
  include_directories (${PROJECT_SOURCE_DIR}/src/win/tinyxml)
  set (tinyxml_LIBRARIES "tinyxml")
  set (tinyxml_LIBRARY_DIRS "")
endif()

################################################
# Find urdfdom parser. Logic:
#
#  1. if USE_INTERNAL_URDF is unset, try to use system installation, fallback to internal copy
#  2. if USE_INTERNAL_URDF is set to True, use the internal copy
#  3. if USE_INTERNAL_URDF is set to False, force to search system installation, fail on error

if (NOT PKG_CONFIG_FOUND)
  if (NOT DEFINED USE_INTERNAL_URDF)
    BUILD_WARNING("Couldn't find pkg-config for urdfdom, using internal copy")
    set(USE_INTERNAL_URDF true)
  elseif(NOT USE_INTERNAL_URDF)
    BUILD_ERROR("Couldn't find pkg-config for urdfdom")
  endif()
endif()

if (NOT DEFINED USE_INTERNAL_URDF OR NOT USE_INTERNAL_URDF)
  # check for urdfdom with pkg-config
  pkg_check_modules(URDF urdfdom>=1.0)

  if (NOT URDF_FOUND)
    if (NOT DEFINED USE_INTERNAL_URDF)
      message(STATUS "Couldn't find urdfdom >= 1.0, using internal copy")
      set(USE_INTERNAL_URDF true)
    else()
      BUILD_ERROR("Couldn't find the urdfdom >= 1.0 system installation")
    endif()
  else()
    # what am I doing here? pkg-config and cmake
    set(URDF_INCLUDE_DIRS ${URDF_INCLUDEDIR})
    set(URDF_LIBRARY_DIRS ${URDF_LIBDIR})
  endif()
endif()

#################################################
# Find ign command line utility:
find_package(ignition-tools)
if (IGNITION-TOOLS_BINARY_DIRS)
  message(STATUS "Looking for ignition-tools-config.cmake - found")
else()
  BUILD_WARNING ("ignition-tools not found, for command line utilities, please install ignition-tools.")
endif()

################################################
# Find the Python interpreter for running the
# check_test_ran.py script
find_package(PythonInterp QUIET)

################################################
# Find psutil python package for memory tests
find_python_module(psutil)
if(NOT PY_PSUTIL)
  BUILD_WARNING("Python psutil package not found. Memory leak tests will be skipped")
endif()

################################################
# Find Valgrind for checking memory leaks in the
# tests
find_program(VALGRIND_PROGRAM NAMES valgrind PATH ${VALGRIND_ROOT}/bin)
option(SDFORMAT_RUN_VALGRIND_TESTS "Run sdformat tests with Valgrind" FALSE)
mark_as_advanced(SDFORMAT_RUN_VALGRIND_TESTS)
if (SDFORMAT_RUN_VALGRIND_TESTS AND NOT VALGRIND_PROGRAM)
  BUILD_WARNING("valgrind not found. Memory check tests will be skipped.")
endif()

################################################
# Find ruby executable to produce xml schemas
find_program(RUBY ruby)
if (NOT RUBY)
    BUILD_ERROR ("Ruby version 1.9 is needed to build xml schemas")
else()
    message(STATUS "Found ruby executable: ${RUBY}")
endif()

#################################################
# Macro to check for visibility capability in compiler
# Original idea from: https://gitorious.org/ferric-cmake-stuff/
macro (check_gcc_visibility)
  include (CheckCXXCompilerFlag)
  check_cxx_compiler_flag(-fvisibility=hidden GCC_SUPPORTS_VISIBILITY)
endmacro()

########################################
# Find ignition math
# Set a variable for generating ProjectConfig.cmake
find_package(ignition-math4 QUIET)
if (NOT ignition-math4_FOUND)
  message(STATUS "Looking for ignition-math4-config.cmake - not found")
  find_package(ignition-math5 QUIET)
  if (NOT ignition-math5_FOUND)
    message(STATUS "Looking for ignition-math5-config.cmake - not found")
    BUILD_ERROR ("Missing: Ignition math (libignition-math4-dev or libignition-math5-dev)")
  else()
    set(IGN_MATH_VER ${ignition-math5_VERSION_MAJOR})
    message(STATUS "Looking for ignition-math5-config.cmake - found")
  endif()
else()
  set(IGN_MATH_VER ${ignition-math4_VERSION_MAJOR})
  message(STATUS "Looking for ignition-math${IGN_MATH_VER}-config.cmake - found")
endif()
