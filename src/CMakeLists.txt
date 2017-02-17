include (${sdf_cmake_dir}/SDFUtils.cmake)

include_directories(
  ${Boost_INCLUDE_DIR}
  ${IGNITION-MATH_INCLUDE_DIRS}
)

link_directories(
  ${PROJECT_BINARY_DIR}/test
  ${Boost_LIBRARY_DIR}
  ${IGNITION-MATH_LIBRARY_DIRS}
)

link_directories(${URDF_LIBRARY_DIRS})

if (USE_EXTERNAL_TINYXML)
  link_directories(${tinyxml_LIBRARY_DIRS})
endif()

set (sources
  Console.cc
  Converter.cc
  Element.cc
  Exception.cc
  parser.cc
  parser_urdf.cc
  Param.cc
  SDF.cc
  SDFExtension.cc
  Types.cc
)

include_directories(${Boost_INCLUDE_DIRS})

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

include_directories(${URDF_INCLUDE_DIRS})

set (gtest_sources
  SDF_TEST.cc
  Console_TEST.cc
  Converter_TEST.cc
  Exception_TEST.cc
  parser_urdf_TEST.cc
  Param_TEST.cc
  parser_TEST.cc
)
sdf_build_tests(${gtest_sources})

sdf_add_library(sdformat ${sources})
target_link_libraries(sdformat ${Boost_LIBRARIES} ${IGNITION-MATH_LIBRARIES})

if (USE_EXTERNAL_TINYXML)
  target_link_libraries(sdformat ${tinyxml_LIBRARIES})
endif()

message (STATUS "URDF_LIBRARY_DIRS=${URDF_LIBRARY_DIRS}")
message (STATUS "URDF_LIBRARIES=${URDF_LIBRARIES}")

target_link_libraries(sdformat ${URDF_LIBRARIES})

sdf_install_library(sdformat)
