include (${sdf_cmake_dir}/SDFUtils.cmake)

include_directories(
  ${Boost_INCLUDE_DIRS}
  ${IGNITION-MATH_INCLUDE_DIRS}
)

link_directories(
  ${PROJECT_BINARY_DIR}/test
  ${IGNITION-MATH_LIBRARY_DIRS}
)

if (NOT USE_INTERNAL_URDF)
  link_directories(${URDF_LIBRARY_DIRS})
endif()

if (USE_EXTERNAL_TINYXML)
  link_directories(${tinyxml_LIBRARY_DIRS})
endif()

set (sources
  Console.cc
  Converter.cc
  Element.cc
  Error.cc
  Exception.cc
  Filesystem.cc
  ign.cc
  parser.cc
  parser_urdf.cc
  Param.cc
  Root.cc
  SDF.cc
  SDFExtension.cc
  Types.cc
  Utils.cc
  World.cc
)

if (USE_EXTERNAL_TINYXML)
  include_directories(${tinyxml_INCLUDE_DIRS})
else()
  set(sources ${sources}
    win/tinyxml/tinystr.cpp
    win/tinyxml/tinyxmlerror.cpp
    win/tinyxml/tinyxml.cpp
    win/tinyxml/tinyxmlparser.cpp)

    install (FILES win/tinyxml/tinyxml.h
	    DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/sdformat-${SDF_VERSION})
endif()

if (USE_INTERNAL_URDF)
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/urdf)
  set(sources ${sources}
  urdf/urdf_parser/model.cpp
  urdf/urdf_parser/link.cpp
  urdf/urdf_parser/joint.cpp
  urdf/urdf_parser/pose.cpp
  urdf/urdf_parser/twist.cpp
  urdf/urdf_parser/urdf_model_state.cpp
  urdf/urdf_parser/urdf_sensor.cpp
  urdf/urdf_parser/world.cpp)
else()
  include_directories(${URDF_INCLUDE_DIRS})
endif()

set (gtest_sources
  SDF_TEST.cc
  Console_TEST.cc
  Converter_TEST.cc
  Element_TEST.cc
  Error_TEST.cc
  Exception_TEST.cc
  Filesystem_TEST.cc
  parser_urdf_TEST.cc
  Param_TEST.cc
  parser_TEST.cc
  Root_TEST.cc
  Types_TEST.cc
  World_TEST.cc
)

# Build this test file only if Ignition Tools is installed.
if (IGNITION-TOOLS_BINARY_DIRS)
  set (gtest_sources ${gtest_sources}
    ign_TEST.cc
  )
endif()

sdf_build_tests(${gtest_sources})

sdf_add_library(sdformat ${sources})
target_link_libraries(sdformat ${IGNITION-MATH_LIBRARIES})

if (USE_EXTERNAL_TINYXML)
  target_link_libraries(sdformat ${tinyxml_LIBRARIES})
else()
  # Ignore the warnings from the internal library
  set_target_properties(sdformat PROPERTIES
      LINK_FLAGS "/IGNORE:4049 /IGNORE:4217")
endif()

message (STATUS "URDF_LIBRARY_DIRS=${URDF_LIBRARY_DIRS}")
message (STATUS "URDF_LIBRARIES=${URDF_LIBRARIES}")

if (NOT USE_INTERNAL_URDF)
  target_link_libraries(sdformat ${URDF_LIBRARIES})
endif()

sdf_install_library(sdformat)

add_subdirectory(cmd)
