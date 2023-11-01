# Collect source files into the "sources" variable and unit test files into the
# "gtest_sources" variable
ign_get_libsources_and_unittests(sources gtest_sources)

# Add the source file auto-generated into the build folder from sdf/CMakeLists.txt
list(APPEND sources EmbeddedSdf.cc)

# When using the internal URDF parser, we build its sources with the core library
if (USE_INTERNAL_URDF)
  set(urdf_internal_sources
    urdf/urdf_parser/model.cpp
    urdf/urdf_parser/link.cpp
    urdf/urdf_parser/joint.cpp
    urdf/urdf_parser/pose.cpp
    urdf/urdf_parser/twist.cpp
    urdf/urdf_parser/urdf_model_state.cpp
    urdf/urdf_parser/urdf_sensor.cpp
    urdf/urdf_parser/world.cpp)
  set(sources ${sources} ${urdf_internal_sources})
endif()

ign_create_core_library(SOURCES ${sources}
  CXX_STANDARD 17
  LEGACY_PROJECT_PREFIX SDFormat
  )

target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}
  PUBLIC
    ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER}
    ignition-utils${IGN_UTILS_VER}::ignition-utils${IGN_UTILS_VER}
  PRIVATE
    TINYXML2::TINYXML2)

  if (USE_INTERNAL_URDF)
    target_include_directories(${PROJECT_LIBRARY_TARGET_NAME} PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/urdf)
    if (WIN32)
      target_compile_definitions(${PROJECT_LIBRARY_TARGET_NAME} PRIVATE -D_USE_MATH_DEFINES)
    endif()
  else()
    target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME} PRIVATE
      IgnURDFDOM::IgnURDFDOM)
  endif()

if (WIN32)
  target_compile_definitions(${PROJECT_LIBRARY_TARGET_NAME} PRIVATE URDFDOM_STATIC)
endif()

target_include_directories(${PROJECT_LIBRARY_TARGET_NAME}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

if (BUILD_TESTING)
  # Build this test file only if Ignition Tools is installed.
  if (NOT IGN_PROGRAM)
    list(REMOVE_ITEM gtest_sources ign_TEST.cc)
  endif()

  add_library(library_for_tests STATIC
      Converter.cc
      EmbeddedSdf.cc
      FrameSemantics.cc
      ParamPassing.cc
      SDFExtension.cc
      Utils.cc
      XmlUtils.cc
      parser.cc
      parser_urdf.cc
  )

  if (USE_INTERNAL_URDF)
    target_sources(library_for_tests PRIVATE ${urdf_internal_sources})
  endif()

  target_link_libraries(library_for_tests PUBLIC
    ${PROJECT_LIBRARY_TARGET_NAME}
    # Also link against the privately linked libraries of the core library
    $<TARGET_PROPERTY:${PROJECT_LIBRARY_TARGET_NAME},LINK_LIBRARIES>
  )

  # Use the private include flags from the core library
  target_include_directories(library_for_tests PUBLIC
    $<TARGET_PROPERTY:${PROJECT_LIBRARY_TARGET_NAME},INCLUDE_DIRECTORIES>
  )

  # Use the private comopile flags from the core library
  target_compile_definitions(library_for_tests PUBLIC
    $<TARGET_PROPERTY:${PROJECT_LIBRARY_TARGET_NAME},COMPILE_DEFINITIONS>
  )

  ign_build_tests(
    TYPE UNIT
    SOURCES ${gtest_sources}
    INCLUDE_DIRS
      ${CMAKE_CURRENT_SOURCE_DIR}
      ${PROJECT_SOURCE_DIR}/test
    LIB_DEPS
      library_for_tests
    TEST_LIST
      test_targets
  )

  # MSVC will fail saying there are some symbols are defined multiple times. This is true
  # since some of the symbols in the static library are public symbols and will also be in the
  # dynamic library (core library). But this is okay since they will be exactly the same definitions,
  # so here, we set `/FORCE:MULTIPLE` to disable the error. A compiler warning is also emitted,
  # so we disable warning C4273.
  if(WIN32)
    foreach(test_target ${test_targets})
      set_target_properties(${test_target} PROPERTIES LINK_FLAGS "/FORCE:MULTIPLE")
    endforeach()

    set_target_properties(library_for_tests PROPERTIES COMPILE_FLAGS "/wd4273")
  endif()

endif()

if(NOT WIN32)
  add_subdirectory(cmd)
endif()
