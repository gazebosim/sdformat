include (${sdf_cmake_dir}/SDFUtils.cmake)

include_directories(
  ${Boost_INCLUDE_DIR}
)

link_directories(
  ${PROJECT_BINARY_DIR}/test
  ${tinyxml_LIBRARY_DIRS}
)

if (USE_EXTERNAL_URDF)
  link_directories(${URDF_LIBRARY_DIRS})
endif()

set (sources
  Assert.cc
  Console.cc
  Converter.cc
  Exception.cc
  parser.cc
  parser_urdf.cc
  Param.cc
  SDF.cc
)

if (USE_EXTERNAL_URDF)
  include_directories(${URDF_INCLUDE_DIRS})
else()
  include_directories(${CMAKE_CURRENT_SOURCE_DIR}/urdf)
  set(sources ${sources}
  urdf/urdf_parser/model.cpp
  urdf/urdf_parser/link.cpp
  urdf/urdf_parser/joint.cpp
  urdf/urdf_parser/pose.cpp
  urdf/urdf_parser/twist.cpp
  urdf/urdf_parser/urdf_model_state.cpp
  urdf/urdf_parser/urdf_sensor.cpp
  urdf/urdf_parser/world.cpp
)
endif()

set (gtest_sources
  SDF_TEST.cc
  Console_TEST.cc
  Converter_TEST.cc
  parser_urdf_TEST.cc
  Param_TEST.cc
)
sdf_build_tests(${gtest_sources})

sdf_add_library(sdformat ${sources})
target_link_libraries(sdformat
  ${tinyxml_LIBRARIES}
  ${Boost_LIBRARIES}
)

message (STATUS "URDF_LIBRARY_DIRS=${URDF_LIBRARY_DIRS}")
message (STATUS "URDF_LIBRARIES=${URDF_LIBRARIES}")

if (USE_EXTERNAL_URDF)
  target_link_libraries(sdformat ${URDF_LIBRARIES})
endif()

sdf_install_library(sdformat)
