set(TEST_TYPE "INTEGRATION")

set(tests
  audio.cc
  fixed_joint_reduction.cc
  provide_feedback.cc
  cfm_damping_implicit_spring_damper.cc
)

link_directories(${PROJECT_BINARY_DIR}/test)

sdf_build_tests(${tests})
