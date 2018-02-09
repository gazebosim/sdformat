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

# We want to embed the sdf XML files directly into the library so we don't
# have to go looking around the filesystem for them in the normal case.  To
# accomplish that, we generate a C++ source and header file pair that contains
# the strings representing the XML data located in the 'sdf' directory.
# The main generator is a python file, which ideally we would pass a list of
# the globbed filenames to.  However, that list is much too long for the
# command-line, so instead we output the list of files to another file, then
# pass the name of *that* filename into the python script.  The python script
# takes the output filename and the input filename containing the list of files
# and generates the C++ and header file necessary for each one.
file(GLOB xml_files ../sdf/**/*.sdf ../sdf/**/*.convert)
file(WRITE "${PROJECT_BINARY_DIR}/filelist.txt" "${xml_files}")
add_custom_command(
  OUTPUT "${PROJECT_BINARY_DIR}/EmbeddedXML.cc"
  COMMAND "${PYTHON_EXECUTABLE}" "${CMAKE_CURRENT_LIST_DIR}/embed_xml_in_cpp_string.py" "${PROJECT_BINARY_DIR}/EmbeddedXML.cc" "${PROJECT_BINARY_DIR}/EmbeddedXML.hh" "${PROJECT_BINARY_DIR}/filelist.txt"
  DEPENDS "${CMAKE_CURRENT_LIST_DIR}/embed_xml_in_cpp_string.py" "${PROJECT_BINARY_DIR}/filelist.txt" "${xml_files}"
  COMMENT "Generate Embedded XML data for the library"
)

set (sources
  Collision.cc
  Console.cc
  Converter.cc
  Element.cc
  "${PROJECT_BINARY_DIR}/EmbeddedXML.cc"
  Error.cc
  Exception.cc
  Filesystem.cc
  ign.cc
  Joint.cc
  Link.cc
  Model.cc
  parser.cc
  parser_urdf.cc
  Param.cc
  Root.cc
  SDF.cc
  SDFExtension.cc
  Types.cc
  Utils.cc
  Visual.cc
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
  Collision_TEST.cc
  Console_TEST.cc
  Converter_TEST.cc
  Element_TEST.cc
  Error_TEST.cc
  Exception_TEST.cc
  Filesystem_TEST.cc
  Joint_TEST.cc
  Link_TEST.cc
  Model_TEST.cc
  parser_urdf_TEST.cc
  Param_TEST.cc
  parser_TEST.cc
  Root_TEST.cc
  SDF_TEST.cc
  Types_TEST.cc
  Visual_TEST.cc
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
