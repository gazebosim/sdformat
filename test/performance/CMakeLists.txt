set(TEST_TYPE "PERFORMANCE")

set(tests
  parser_urdf.cc
)

link_directories(${PROJECT_BINARY_DIR}/test)

sdf_build_tests(${tests})
