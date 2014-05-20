include (${sdf_cmake_dir}/SDFUtils.cmake)
include (${sdf_cmake_dir}/FindOS.cmake)
include (FindPkgConfig)

########################################
# Find Boost, if not specified manually

if (WIN32)
    # The following verifyies that BOOST_ROOT is set properly.
    if(NOT BOOST_ROOT AND NOT $ENV{BOOST_ROOT} STREQUAL "")
        FILE( TO_CMAKE_PATH $ENV{BOOST_ROOT} BOOST_ROOT )
        if( NOT EXISTS ${BOOST_ROOT} ) 
            MESSAGE( STATUS  ${BOOST_ROOT} " does not exist. Checking if BOOST_ROOT was a quoted string.." )
            STRING( REPLACE "\"" "" BOOST_ROOT ${BOOST_ROOT} ) 
            if( EXISTS ${BOOST_ROOT} ) 
                MESSAGE( STATUS "After removing the quotes " ${BOOST_ROOT} " was now found by CMake" )
            endif( EXISTS ${BOOST_ROOT})
        endif( NOT EXISTS ${BOOST_ROOT} )

	# Save the BOOST_ROOT in the cache
	if( NOT EXISTS ${BOOST_ROOT} ) 
	    MESSAGE( WARNING ${BOOST_ROOT} " does not exist." )
	else(NOT EXISTS ${BOOST_ROOT})
	    SET (BOOST_ROOT ${BOOST_ROOT} CACHE STRING "Set the value of BOOST_ROOT to point to the root folder of your boost install." FORCE)
	endif( NOT EXISTS ${BOOST_ROOT} )
    endif(NOT BOOST_ROOT AND NOT $ENV{BOOST_ROOT} STREQUAL "")

    if(BOOST_ROOT)
        MESSAGE(WARNING "Please set the BOOST_ROOT environment variable.")
    endif()

    set(Boost_DEBUG ON)
    set(Boost_USE_STATIC_LIBS        ON)
    set(Boost_USE_MULTITHREADED      OFF)
    set(Boost_USE_STATIC_RUNTIME     ON)
endif()

include(FindBoost)
find_package(Boost ${MIN_BOOST_VERSION} REQUIRED system filesystem program_options regex iostreams)

#FIND_PACKAGE(Boost 1.47.0 COMPONENTS ${BOOST_COMPONENTS_NEEDED})
if(Boost_FOUND)
    MESSAGE( STATUS "Setting up boost." )
    include_directories(${Boost_INCLUDE_DIRS})
    if(Boost_DEBUG) 
	MESSAGE( STATUS "BOOST Libraries " ${Boost_LIBRARIES} )
	FOREACH(BOOST_COMPONENT ${BOOST_COMPONENTS_NEEDED})
	    STRING( TOUPPER ${BOOST_COMPONENT} BOOST_COMPONENT_UPCASE )
	    MESSAGE( STATUS "Boost " ${BOOST_COMPONENT} ": " ${Boost_${BOOST_COMPONENT_UPCASE}_LIBRARY} )
	    MESSAGE( STATUS "Boost " ${BOOST_COMPONENT} " Debug: " ${Boost_${BOOST_COMPONENT_UPCASE}_LIBRARY_DEBUG} )
	    MESSAGE( STATUS "Boost " ${BOOST_COMPONENT} " Release: " ${Boost_${BOOST_COMPONENT_UPCASE}_LIBRARY_RELEASE} )
	ENDFOREACH(BOOST_COMPONENT)
    endif(Boost_DEBUG)
endif(Boost_FOUND)

if (NOT Boost_FOUND)
  set (BUILD_SDF OFF CACHE INTERNAL "Build SDF" FORCE)
  BUILD_ERROR ("Boost not found. Please install thread signals system filesystem program_options regex boost version ${MIN_BOOST_VERSION} or higher.")
endif() 

if (UNIX)
  set (USE_EXTERNAL_TINXYML:BOOL True)
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
  set (USE_EXTERNAL_TINXYML:BOOL False)
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
