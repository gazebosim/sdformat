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
find_path (tinyxml_include_dir tinyxml.h ${tinyxml_include_dirs} ENV CPATH)
if (NOT tinyxml_include_dir)
  message (STATUS "Looking for tinyxml.h - not found")
  BUILD_ERROR("Missing: tinyxml")
else ()
  message (STATUS "Looking for tinyxml.h - found")
  set (tinyxml_include_dirs ${tinyxml_include_dir} CACHE STRING
    "tinyxml include paths. Use this to override automatic detection.")
  set (tinyxml_libraries "tinyxml" CACHE INTERNAL "tinyxml libraries")
endif ()

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
