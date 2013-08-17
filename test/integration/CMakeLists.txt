set(TEST_TYPE "INTEGRATION")

set(tests
  audio.cc
)

link_directories(${PROJECT_BINARY_DIR}/test)

sdf_build_tests(${tests})
