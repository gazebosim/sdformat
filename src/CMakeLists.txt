include (${sdf_cmake_dir}/SDFUtils.cmake)

link_directories(
  ${PROJECT_BINARY_DIR}/test
)

set (sources
  Assert.cc
  Console.cc
  Converter.cc
  Exception.cc
  parser.cc
  Param.cc
  SDF.cc
)

set (gtest_sources
  SDF_TEST.cc
  Converter_TEST.cc
)

sdf_build_tests(${gtest_sources})

if (HAVE_URDFDOM AND HAVE_URDFDOM_HEADERS)
  include_directories(SYSTEM ${urdfdom_INCLUDE_DIRS}
                             ${urdfdom_headers_INCLUDE_DIRS}
                             ${console_bridge_INCLUDE_DIRS}
                             )

  link_directories(${urdfdom_LIBRARY_DIRS}
                   ${urdfdom_headers_LIBRARY_DIRS}
                   ${console_bridge_LIBRARY_DIRS})

  set (sources ${sources} parser_urdf.cc)

  # add_definitions(${urdfdom_LDFLAGS}
  # ${urdfdom_headers_LDFLAGS} ${console_bridge_LDFLAGS})
endif(HAVE_URDFDOM AND HAVE_URDFDOM_HEADERS)

sdf_add_library(sdf ${sources})

if (HAVE_URDFDOM AND HAVE_URDFDOM_HEADERS)
  target_link_libraries(sdf
    ${tinyxml_libraries}
    ${Boost_LIBRARIES}
    ${urdfdom_headers_LIBRARIES}
    ${urdfdom_LIBRARIES}
    ${console_bridge_LIBRARIES}
    ${urdfdom_LDFLAGS}
    ${console_bridge_LDFLAGS})
else(HAVE_URDFDOM AND HAVE_URDFDOM_HEADERS)
  target_link_libraries(sdf
    ${tinyxml_libraries}
    ${Boost_LIBRARIES})
endif(HAVE_URDFDOM AND HAVE_URDFDOM_HEADERS)

sdf_install_library(sdf)
