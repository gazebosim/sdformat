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
  Box.cc
  Collision.cc
  Console.cc
  Converter.cc
  Cylinder.cc
  Element.cc
  Error.cc
  Exception.cc
  Filesystem.cc
  Geometry.cc
  Gui.cc
  ign.cc
  Joint.cc
  JointAxis.cc
  Link.cc
  Material.cc
  Mesh.cc
  Model.cc
  parser.cc
  parser_urdf.cc
  Param.cc
  Plane.cc
  Root.cc
  SDF.cc
  SDFExtension.cc
  Sphere.cc
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
  Box_TEST.cc
  Collision_TEST.cc
  Console_TEST.cc
  Converter_TEST.cc
  Cylinder_TEST.cc
  Element_TEST.cc
  Error_TEST.cc
  Exception_TEST.cc
  Filesystem_TEST.cc
  Geometry_TEST.cc
  Gui_TEST.cc
  Joint_TEST.cc
  JointAxis_TEST.cc
  Link_TEST.cc
  Material_TEST.cc
  Mesh_TEST.cc
  Model_TEST.cc
  parser_urdf_TEST.cc
  Param_TEST.cc
  parser_TEST.cc
  Plane_TEST.cc
  Root_TEST.cc
  SDF_TEST.cc
  Sphere_TEST.cc
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
