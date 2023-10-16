# Collect source files into the "sources" variable and unit test files into the
# "gtest_sources" variable
gz_get_libsources_and_unittests(sources gtest_sources)

# Add the source file auto-generated into the build folder from sdf/CMakeLists.txt
list(APPEND sources EmbeddedSdf.cc)

# Use interface library to deduplicate cmake logic for URDF linking
add_library(using_parser_urdf INTERFACE)
if (USE_INTERNAL_URDF)
  set(sources ${sources}
    urdf/urdf_parser/model.cpp
    urdf/urdf_parser/link.cpp
    urdf/urdf_parser/joint.cpp
    urdf/urdf_parser/pose.cpp
    urdf/urdf_parser/twist.cpp
    urdf/urdf_parser/urdf_model_state.cpp
    urdf/urdf_parser/urdf_sensor.cpp
    urdf/urdf_parser/world.cpp)
  target_include_directories(using_parser_urdf INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/urdf)
  if (WIN32)
    target_compile_definitions(using_parser_urdf INTERFACE -D_USE_MATH_DEFINES)
  endif()
else()
  target_link_libraries(using_parser_urdf INTERFACE
    GzURDFDOM::GzURDFDOM)
endif()

if (BUILD_TESTING)
  # Build this test file only if Gazebo Tools is installed.
  if (NOT GZ_PROGRAM)
    list(REMOVE_ITEM gtest_sources gz_TEST.cc)
  endif()

  # Skip tests that don't work on Windows
  if (WIN32)
    list(REMOVE_ITEM gtest_sources Converter_TEST.cc)
    list(REMOVE_ITEM gtest_sources FrameSemantics_TEST.cc)
    list(REMOVE_ITEM gtest_sources ParamPassing_TEST.cc)
    list(REMOVE_ITEM gtest_sources Utils_TEST.cc)
    list(REMOVE_ITEM gtest_sources XmlUtils_TEST.cc)
    list(REMOVE_ITEM gtest_sources parser_urdf_TEST.cc)
  endif()

  gz_build_tests(
    TYPE UNIT
    SOURCES ${gtest_sources}
    INCLUDE_DIRS
      ${CMAKE_CURRENT_SOURCE_DIR}
      ${PROJECT_SOURCE_DIR}/test
  )

  if (TARGET UNIT_Converter_TEST)
    target_link_libraries(UNIT_Converter_TEST
      TINYXML2::TINYXML2)
    target_sources(UNIT_Converter_TEST PRIVATE
      Converter.cc
      EmbeddedSdf.cc
      Utils.cc
      XmlUtils.cc)
  endif()

  if (TARGET UNIT_FrameSemantics_TEST)
    target_sources(UNIT_FrameSemantics_TEST PRIVATE FrameSemantics.cc Utils.cc)
    target_link_libraries(UNIT_FrameSemantics_TEST TINYXML2::TINYXML2)
  endif()

  if (TARGET UNIT_ParamPassing_TEST)
    target_link_libraries(UNIT_ParamPassing_TEST
      TINYXML2::TINYXML2
      using_parser_urdf)
    target_sources(UNIT_ParamPassing_TEST PRIVATE
      Converter.cc
      EmbeddedSdf.cc
      FrameSemantics.cc
      ParamPassing.cc
      SDFExtension.cc
      Utils.cc
      XmlUtils.cc
      parser.cc
      parser_urdf.cc)
  endif()

  if (TARGET UNIT_Utils_TEST)
    target_sources(UNIT_Utils_TEST PRIVATE Utils.cc)
    target_link_libraries(UNIT_Utils_TEST TINYXML2::TINYXML2)
  endif()

  if (TARGET UNIT_XmlUtils_TEST)
    target_link_libraries(UNIT_XmlUtils_TEST
      TINYXML2::TINYXML2)
    target_sources(UNIT_XmlUtils_TEST PRIVATE XmlUtils.cc)
  endif()

  if (TARGET UNIT_parser_urdf_TEST)
    target_link_libraries(UNIT_parser_urdf_TEST
      TINYXML2::TINYXML2
      using_parser_urdf)
    target_sources(UNIT_parser_urdf_TEST PRIVATE
      SDFExtension.cc
      XmlUtils.cc
      parser_urdf.cc)
  endif()
endif()

gz_create_core_library(SOURCES ${sources}
  CXX_STANDARD 17
  LEGACY_PROJECT_PREFIX SDFormat
  )

target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}
  PUBLIC
    gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER}
    gz-utils${GZ_UTILS_VER}::gz-utils${GZ_UTILS_VER}
  PRIVATE
    TINYXML2::TINYXML2
    using_parser_urdf)

if (WIN32)
  target_compile_definitions(${PROJECT_LIBRARY_TARGET_NAME} PRIVATE URDFDOM_STATIC)
endif()

target_include_directories(${PROJECT_LIBRARY_TARGET_NAME}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

add_subdirectory(cmd)
