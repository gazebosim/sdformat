set(TEST_TYPE "PERFORMANCE")

set(tests
  parser_urdf.cc
)

gz_build_tests(TYPE ${TEST_TYPE} SOURCES ${tests} INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/test)
