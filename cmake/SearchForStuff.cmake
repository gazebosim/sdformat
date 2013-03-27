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
