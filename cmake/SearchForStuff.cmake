include (FindPkgConfig)

# Detect the architecture
include (${project_cmake_dir}/TargetArch.cmake)
target_architecture(ARCH)
message(STATUS "Building for arch: ${ARCH}")

#################################################
# Find ign-cmake
find_package(ignition-cmake2 2.9 REQUIRED)
set(IGN_CMAKE_VER ${ignition-cmake2_VERSION_MAJOR})

#################################################
# Find tinyxml2.
list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")
find_package(TinyXML2 REQUIRED)

################################################
# Find urdfdom parser. Logic:
#
#  1. if USE_INTERNAL_URDF is unset, try to use system installation, fallback to internal copy
#  2. if USE_INTERNAL_URDF is set to True, use the internal copy
#  3. if USE_INTERNAL_URDF is set to False, force to search system installation, fail on error

if (NOT PKG_CONFIG_FOUND)
  if (NOT DEFINED USE_INTERNAL_URDF)
    ign_build_warning("Couldn't find pkg-config for urdfdom, using internal copy")
    set(USE_INTERNAL_URDF true)
  elseif(NOT USE_INTERNAL_URDF)
    ign_build_error("Couldn't find pkg-config for urdfdom")
  endif()
endif()

if (NOT DEFINED USE_INTERNAL_URDF OR NOT USE_INTERNAL_URDF)
  # check for urdfdom with pkg-config
  pkg_check_modules(URDF urdfdom>=1.0)

  if (NOT URDF_FOUND)
    find_package(urdfdom QUIET)
    if (urdfdom_FOUND)
      set(URDF_INCLUDE_DIRS ${urdfdom_INCLUDE_DIRS})
      # ${urdfdom_LIBRARIES} already contains absolute library filenames
      set(URDF_LIBRARY_DIRS "")
      set(URDF_LIBRARIES ${urdfdom_LIBRARIES})
    elseif (NOT DEFINED USE_INTERNAL_URDF)
      message(STATUS "Couldn't find urdfdom >= 1.0, using internal copy")
      set(USE_INTERNAL_URDF true)
    else()
      ign_build_error("Couldn't find the urdfdom >= 1.0 system installation")
    endif()
  else()
    # what am I doing here? pkg-config and cmake
    set(URDF_INCLUDE_DIRS ${URDF_INCLUDEDIR})
    set(URDF_LIBRARY_DIRS ${URDF_LIBDIR})
  endif()
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
find_python_module(psutil)
if(NOT PY_PSUTIL)
  ign_build_warning("Python psutil package not found. Memory leak tests will be skipped")
endif()

################################################
# Find Valgrind for checking memory leaks in the
# tests
find_program(VALGRIND_PROGRAM NAMES valgrind PATH ${VALGRIND_ROOT}/bin)
option(SDFORMAT_RUN_VALGRIND_TESTS "Run sdformat tests with Valgrind" FALSE)
mark_as_advanced(SDFORMAT_RUN_VALGRIND_TESTS)
if (SDFORMAT_RUN_VALGRIND_TESTS AND NOT VALGRIND_PROGRAM)
  ign_build_warning("valgrind not found. Memory check tests will be skipped.")
endif()

################################################
# Find ruby executable to produce xml schemas
find_program(RUBY ruby)
if (NOT RUBY)
    ign_build_error ("Ruby version 1.9 is needed to build xml schemas")
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
# Find ignition cmake2
find_package(ignition-cmake2 2.9 REQUIRED)


########################################
# Find ignition math
# Set a variable for generating ProjectConfig.cmake
ign_find_package(ignition-math6 VERSION 6.8 QUIET)
set(IGN_MATH_VER ${ignition-math6_VERSION_MAJOR})

########################################
# Find ignition utils
# Set a variable for generating ProjectConfig.cmake
ign_find_package(ignition-utils1 QUIET)
set(IGN_UTILS_VER ${ignition-utils1_VERSION_MAJOR})
