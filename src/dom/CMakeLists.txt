include (${sdf_cmake_dir}/SDFUtils.cmake)

set (gtest_sources
  Entity_TEST.cc
  Joint_TEST.cc
)
sdf_build_tests(${gtest_sources})


