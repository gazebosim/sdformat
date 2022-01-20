# Collect source files into the "sources" variable and unit test files into the
# "gtest_sources" variable.
ign_get_libsources_and_unittests(sources gtest_sources)

set(sources ${sources}
  usd_parser/USDData.cc
  usd_parser/USDStage.cc
  usd_parser/utils.cc
)

set(gtest_sources ${gtest_sources}
  usd_parser/USDData_TEST.cc
  usd_parser/USDStage_TEST.cc
)

ign_add_component(usd SOURCES ${sources} GET_TARGET_NAME usd_target)

target_include_directories(${usd_target}
  PUBLIC
    ${PXR_INCLUDE_DIRS}
)

target_link_libraries(${usd_target}
  PUBLIC
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ${PXR_LIBRARIES}
)

# Build the unit tests
ign_build_tests(
  TYPE UNIT
  SOURCES ${gtest_sources}
  LIB_DEPS ${usd_target} ignition-cmake${IGN_CMAKE_VER}::utilities
  INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/test
)

add_subdirectory(cmd)
