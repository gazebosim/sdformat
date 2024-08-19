# Collect source files into the "sources" variable and unit test files into the
# "gtest_sources" variable
gz_get_libsources_and_unittests(sources gtest_sources)

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

gz_create_core_library(SOURCES ${sources}
  CXX_STANDARD 17
  LEGACY_PROJECT_PREFIX SDFormat
  )
gz_add_get_install_prefix_impl(GET_INSTALL_PREFIX_FUNCTION sdf::getInstallPrefix
  GET_INSTALL_PREFIX_HEADER sdf/InstallationDirectories.hh
  OVERRIDE_INSTALL_PREFIX_ENV_VARIABLE SDF_INSTALL_PREFIX)

# CMAKE_INSTALL_DATAROOTDIR may be an absolute path, let's make sure to use the
# relative version
if(IS_ABSOLUTE "${CMAKE_INSTALL_DATAROOTDIR}")
  file(RELATIVE_PATH CMAKE_INSTALL_RELATIVE_DATAROOTDIR "${CMAKE_INSTALL_PREFIX}" "${CMAKE_INSTALL_DATAROOTDIR}")
else()
  set(CMAKE_INSTALL_RELATIVE_DATAROOTDIR "${CMAKE_INSTALL_DATAROOTDIR}")
endif()

set_property(
  SOURCE InstallationDirectories.cc
  PROPERTY COMPILE_DEFINITIONS
  CMAKE_INSTALL_RELATIVE_DATAROOTDIR="${CMAKE_INSTALL_RELATIVE_DATAROOTDIR}"
)


target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}
  PUBLIC
    gz-math${GZ_MATH_VER}::gz-math${GZ_MATH_VER}
    gz-utils${GZ_UTILS_VER}::gz-utils${GZ_UTILS_VER}
  PRIVATE
    TINYXML2::TINYXML2)

if (WIN32)
  target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}
    PRIVATE shlwapi)
endif()

  if (USE_INTERNAL_URDF)
    target_include_directories(${PROJECT_LIBRARY_TARGET_NAME} PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/urdf)
    if (WIN32)
      target_compile_definitions(${PROJECT_LIBRARY_TARGET_NAME} PRIVATE -D_USE_MATH_DEFINES)
    endif()
  else()
    target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME} PRIVATE
      GzURDFDOM::GzURDFDOM)
  endif()

if (WIN32 AND USE_INTERNAL_URDF)
  target_compile_definitions(${PROJECT_LIBRARY_TARGET_NAME} PRIVATE URDFDOM_STATIC)
endif()

target_include_directories(${PROJECT_LIBRARY_TARGET_NAME}
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
)

if (BUILD_TESTING)
  # Build this test file only if Gazebo Tools is installed.
  if (NOT GZ_PROGRAM)
    list(REMOVE_ITEM gtest_sources gz_TEST.cc)
  endif()

  add_library(library_for_tests OBJECT
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

  # Link against the publicly and privately linked libraries of the core library
  target_link_libraries(library_for_tests
    $<TARGET_PROPERTY:${PROJECT_LIBRARY_TARGET_NAME},LINK_LIBRARIES>
  )

  # Use the include flags from the core library
  target_include_directories(library_for_tests PUBLIC
    $<TARGET_PROPERTY:${PROJECT_LIBRARY_TARGET_NAME},INCLUDE_DIRECTORIES>
  )

  # Use the private compile flags from the core library. Also set GZ_SDFORMAT_STATIC_DEFINE to avoid
  # inconsistent linkage issues on windows. Setting the define will cause the SDFORMAT_VISIBLE/SDFORMAT_HIDDEN  macros
  # to expand to nothing when building a static library
  target_compile_definitions(library_for_tests PUBLIC
    $<TARGET_PROPERTY:${PROJECT_LIBRARY_TARGET_NAME},COMPILE_DEFINITIONS>
    -DGZ_SDFORMAT_STATIC_DEFINE
  )

  gz_build_tests(
    TYPE UNIT
    SOURCES ${gtest_sources}
    INCLUDE_DIRS
      ${CMAKE_CURRENT_SOURCE_DIR}
      ${PROJECT_SOURCE_DIR}/test
    LIB_DEPS
      library_for_tests
    ENVIRONMENT
      SDF_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
  )

  if (TARGET UNIT_gz_TEST)
    target_compile_definitions(UNIT_gz_TEST PRIVATE
      -DGZ_PATH="${GZ_PROGRAM}"
      -DDETAIL_GZ_CONFIG_PATH="${CMAKE_BINARY_DIR}/test/conf/$<CONFIG>"
      -DGZ_TEST_LIBRARY_PATH="${PROJECT_BINARY_DIR}/src")
  endif()
endif()

add_subdirectory(cmd)
