configure_file(test_config.h.in ${PROJECT_BINARY_DIR}/include/test_config.h)

# Build gtest
add_library(gtest STATIC gtest/src/gtest-all.cc)
target_include_directories(gtest
  PUBLIC
    ${PROJECT_SOURCE_DIR}/test/gtest/include
    ${PROJECT_SOURCE_DIR}/test/gtest
)
add_library(gtest_main STATIC gtest/src/gtest_main.cc)
target_link_libraries(gtest_main gtest)

execute_process(COMMAND cmake -E remove_directory ${CMAKE_BINARY_DIR}/test_results)
execute_process(COMMAND cmake -E make_directory ${CMAKE_BINARY_DIR}/test_results)

add_subdirectory(integration)
add_subdirectory(performance)
