include (${sdf_cmake_dir}/SDFUtils.cmake)
include (${sdf_cmake_dir}/FindOS.cmake)
include (FindPkgConfig)

########################################
# Find Boost, if not specified manually
include(FindBoost)
find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem program_options regex iostreams)

if (NOT Boost_FOUND)
  set (BUILD_SDF OFF CACHE INTERNAL "Build SDF" FORCE)
  BUILD_ERROR ("Boost not found. Please install thread signals system filesystem program_options regex boost version ${MIN_BOOST_VERSION} or higher.")
endif() 

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

################################################
# Find urdfdom parser
if (USE_EXTERNAL_URDF)
  if (NOT PKG_CONFIG_FOUND)
      BUILD_ERROR ("pkgconfig not found. Please install to so USE_EXTERNAL_URDF can found the urdf library")
  else()
    message(STATUS ${URDF_INCLUDE_DIRS})
    pkg_check_modules(URDF urdfdom)
    message(STATUS ${URDF_INCLUDE_DIRS})
    if (NOT URDF_FOUND)
      BUILD_ERROR ("URDF library not found. Please install it to use with USE_EXTERNAL_URDF or set this flag to false to use internal URDF code")
    endif()
  endif()
endif()

#################################################
# Macro to check for visibility capability in compiler
# Original idea from: https://gitorious.org/ferric-cmake-stuff/ 
macro (check_gcc_visibility)
  include (CheckCXXCompilerFlag)
  check_cxx_compiler_flag(-fvisibility=hidden GCC_SUPPORTS_VISIBILITY)
endmacro()
