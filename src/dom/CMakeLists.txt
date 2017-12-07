include (${sdf_cmake_dir}/SDFUtils.cmake)

set (gtest_sources
  Entity_TEST.cc
  Joint_TEST.cc
  Link_TEST.cc
  Model_TEST.cc
  Root_TEST.cc
  SphericalCoordinates_TEST.cc
  World_TEST.cc
)

sdf_build_tests(${gtest_sources})
