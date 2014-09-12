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

################################################
# Finding Ruby is complicated. We want to avoid finding the version of Ruby
# that is installed via rbenv because the library is not usually in the 
# LIBRARY_PATH. Unfortunately, the FindRuby.cmake will return the version
# of Ruby installed locally using rbenv. So, we first try to use pkg-config.
# If pkg-config fails, then we try find_library and find_path. If that fails,
# then we try find_package.

set (ruby_versions 1.9 1.8)

foreach (ver ${ruby_versions})
  # Check if pkg-config finds ruby
  pkg_check_modules(ruby ruby-${ver})
  if (NOT ruby_FOUND)
    if (${ver} EQUAL 1.9)
      set (ver_full 1.9.1)
    elseif(${ver} EQUAL 1.8)
      set (ver_full 1.8.0)
    endif()

    # pkg-config failed, so try using find_library and find_path
    find_library(RUBY_LIBRARY NAMES ruby-${ver_full})
    find_path(RUBY_INCLUDE_DIRS NAMES ruby.h
      PATHS /usr/include/ruby-${ver_full})

    # The ruby library includes a config.h in a very strange location. This
    # is a "nice" hack to make sure config.h is found.
    set(RUBY_INCLUDE_DIRS "${RUBY_INCLUDE_DIRS};${RUBY_INCLUDE_DIRS}/x86_64-linux")

    # if find_library and find_path failed, try using find_package
    if (NOT RUBY_LIBRARY)
      find_package(Ruby ${ver})

      # Make sure we don't count the static version.
      if (NOT RUBY_FOUND OR "${RUBY_LIBRARY}" MATCHES ".*-static.a")
        set (RUBY_LIBRARY RUBY_LIBRARY-NOTFOUND)
        set (RUBY_INCLUDE_DIRS RUBY_INCLUDE_DIRS-NOTFOUND)
      endif()
    endif()

  else ()
    set (RUBY_LIBRARY ${ruby_LIBRARIES})
    set (RUBY_INCLUDE_DIRS ${ruby_INCLUDE_DIRS})
  endif()

  # Break if ruby was found
  if (RUBY_LIBRARY AND RUBY_INCLUDE_DIRS)
    break()
  endif()
endforeach()

# Generate error if ruby was not found
if (NOT RUBY_LIBRARY OR NOT RUBY_INCLUDE_DIRS)
  message(STATUS "Looking for libruby - not found")
  BUILD_ERROR("Ruby (ruby-dev) is required to parse ERB files.")
else()
  message(Status "Looking for ruby - found")
endif()
