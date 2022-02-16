set(sources
  UsdError.cc
  sdf_parser/Light.cc
  sdf_parser/World.cc
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

set(gtest_sources
  sdf_parser/sdf2usd_TEST.cc
  sdf_parser/Light_Sdf2Usd_TEST.cc
  sdf_parser/World_Sdf2Usd_TEST.cc
  UsdError_TEST.cc
  UsdUtils_TEST.cc
)

# Build the unit tests
ign_build_tests(
  TYPE UNIT
  SOURCES ${gtest_sources}
  LIB_DEPS ${usd_target} ignition-cmake${IGN_CMAKE_VER}::utilities
  INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/test
)

add_subdirectory(cmd)
