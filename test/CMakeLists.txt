configure_file(test_config.hh.in ${PROJECT_BINARY_DIR}/include/test_config.hh)
include_directories(
  ${PROJECT_BINARY_DIR}/include
)

add_subdirectory(gtest_vendor)
add_subdirectory(integration)
add_subdirectory(performance)
