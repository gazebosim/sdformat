include (${sdf_cmake_dir}/FindOS.cmake)
include (FindPkgConfig)

########################################
# Find Boost, if not specified manually
if (WIN32)
  set(Boost_USE_STATIC_LIBS       OFF) 
  set(Boost_USE_MULTITHREADED      ON)
  set(Boost_USE_STATIC_RUNTIME    OFF)
endif()

include(FindBoost)
find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem program_options regex iostreams)

if (NOT Boost_FOUND)
  set (BUILD_SDF OFF CACHE INTERNAL "Build SDF" FORCE)
  BUILD_ERROR ("Boost not found. Please install thread signals system filesystem program_options regex boost version ${MIN_BOOST_VERSION} or higher.")
endif() 

if (USE_EXTERNAL_TINXYML)
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
# Find urdfdom parser
if (USE_EXTERNAL_URDF)
  if (NOT PKG_CONFIG_FOUND)
      BUILD_ERROR ("pkgconfig not found. Please install to so USE_EXTERNAL_URDF can found the urdf library")
  else()
    # check for urdfdom with pkg-config
    pkg_check_modules(URDF urdfdom>=0.3)

    if (NOT URDF_FOUND)
      # version >= 0.3.x not found, check for urdfdom again with no version
      # restriction.
      pkg_check_modules(URDF urdfdom)


      if (NOT URDF_FOUND)
        BUILD_ERROR ("URDF library not found. Please install it to use with USE_EXTERNAL_URDF or set this flag to false to use internal URDF code")
      else()
        # urdfdom library found < 0.3, unset flag
        set (URDF_GE_0P3 FALSE)
      endif()

    else()
      # urdfdom library found >= 0.3, set flag
      set (URDF_GE_0P3 TRUE)
    endif()

    # what am I doing here? pkg-config and cmake
    set(URDF_INCLUDE_DIRS ${URDF_INCLUDEDIR})
    set(URDF_LIBRARY_DIRS ${URDF_LIBDIR})
  endif()
endif()

#################################################
# Macro to check for visibility capability in compiler
# Original idea from: https://gitorious.org/ferric-cmake-stuff/ 
macro (check_gcc_visibility)
  include (CheckCXXCompilerFlag)
  check_cxx_compiler_flag(-fvisibility=hidden GCC_SUPPORTS_VISIBILITY)
endmacro()
