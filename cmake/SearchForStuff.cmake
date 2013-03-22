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


if (PKG_CONFIG_FOUND)

  ########################################
  # Find urdfdom and urdfdom_headers
  pkg_check_modules(urdfdom_headers urdfdom_headers)
  if (NOT urdfdom_headers_FOUND)
    BUILD_WARNING ("urdfdom_headers not found, urdf parser will not be built.")
  endif ()
  if (urdfdom_headers_FOUND)
    set (HAVE_URDFDOM_HEADERS TRUE)
  endif ()
  
  pkg_check_modules(urdfdom urdfdom)
  if (NOT urdfdom_FOUND)
    BUILD_WARNING ("urdfdom not found, urdf parser will not be built.")
  endif ()
  if (urdfdom_FOUND)
    set (HAVE_URDFDOM TRUE)
  endif ()
  
  pkg_check_modules(console_bridge console_bridge)
  if (NOT console_bridge_FOUND)
    BUILD_WARNING ("console_bridge not found, urdf parser will not be built.")
  endif ()
  if (console_bridge_FOUND)
    set (HAVE_CONSOLE_BRIDGE TRUE)
  endif ()
endif(PKG_CONFIG_FOUND)

########################################
# Find urdfdom_headers
if (NOT HAVE_URDFDOM_HEADERS)
  set (urdfdom_search_path /usr/include)
  find_path(URDFDOM_HEADERS_PATH urdf_model/model.h ${urdfdom_search_path})
  if (NOT URDFDOM_HEADERS_PATH)
    message (STATUS "Looking for urdf_model/model.h - not found")
    BUILD_WARNING ("model.h not found. urdf parser will not be built")
  else (NOT URDFDOM_HEADERS_PATH)
    message (STATUS "Looking for model.h - found")
    set (HAVE_URDFDOM_HEADERS TRUE)
    set (URDFDOM_HEADERS_PATH /usr/include)
  endif (NOT URDFDOM_HEADERS_PATH)
else (NOT HAVE_URDFDOM_HEADERS)
  set (URDFDOM_HEADERS_PATH /usr/include)
  message (STATUS "found urdf_model/model.h - found")
endif (NOT HAVE_URDFDOM_HEADERS)

########################################
# Find urdfdom
if (NOT HAVE_URDFDOM)
  set (urdfdom_search_path 
    /usr/include /usr/local/include 
    /usr/include/urdf_parser
    )

  find_path(URDFDOM_PATH urdf_parser.h ${urdfdom_search_path})
  if (NOT URDFDOM_PATH)
    message (STATUS "Looking for urdf_parser/urdf_parser.h - not found")
    BUILD_WARNING ("urdf_parser.h not found. urdf parser will not be built")
    set (URDFDOM_PATH /usr/include)
  else (NOT URDFDOM_PATH)
    message (STATUS "Looking for urdf_parser.h - found")
    set (HAVE_URDFDOM TRUE)
    set (URDFDOM_PATH /usr/include)
  endif (NOT URDFDOM_PATH)

else (NOT HAVE_URDFDOM)
  message (STATUS "found urdf_parser/urdf_parser.h - found")
endif (NOT HAVE_URDFDOM)

########################################
# Find console_bridge
if (NOT HAVE_CONSOLE_BRIDGE)
  set (console_bridge_search_path 
    /usr/include /usr/local/include 
    )

  find_path(CONSOLE_BRIDGE_PATH console_bridge/console.h ${console_bridge_search_path})
  if (NOT CONSOLE_BRIDGE_PATH)
    message (STATUS "Looking for console_bridge/console.h - not found")
    BUILD_WARNING ("console.h not found. urdf parser (depends on console_bridge) will not be built")
    set (CONSOLE_BRIDGE_PATH /usr/include)
  else (NOT CONSOLE_BRIDGE_PATH)
    message (STATUS "Looking for console.h - found")
    set (HAVE_CONSOLE_BRIDGE TRUE)
    set (CONSOLE_BRIDGE_PATH /usr/include)
  endif (NOT CONSOLE_BRIDGE_PATH)

else (NOT HAVE_CONSOLE_BRIDGE)
  message (STATUS "found console_bridge/console.h - found")
endif (NOT HAVE_CONSOLE_BRIDGE)
