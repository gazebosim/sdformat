# Collect source files into the "sources" variable and unit test files into the
# "gtest_sources" variable.
ign_get_libsources_and_unittests(sources gtest_sources)

ign_add_component(usd SOURCES ${sources} GET_TARGET_NAME usd_target)

target_include_directories(${usd_target}
  PUBLIC
    ${PXR_INCLUDE_DIRS}
)

target_link_libraries(${usd_target}
  PUBLIC
    ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
    ignition-common${IGN_COMMON_VER}::graphics
    ${PXR_LIBRARIES}
)

# Build the unit tests
include_directories(${PROJECT_SOURCE_DIR}/test)
ign_build_tests(TYPE UNIT SOURCES ${gtest_sources} LIB_DEPS ${usd_target})

add_subdirectory(cmd)
