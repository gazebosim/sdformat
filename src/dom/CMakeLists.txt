include (${sdf_cmake_dir}/SDFUtils.cmake)

set(sources ${sources}
  dom/Root.cc
  PARENT_SCOPE
)

set (gtest_sources
  Root_TEST.cc
)

sdf_build_tests(${gtest_sources})
