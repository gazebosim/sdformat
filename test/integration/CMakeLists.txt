set(TEST_TYPE "INTEGRATION")

set(tests
  audio.cc
  cfm_damping_implicit_spring_damper.cc
  fixed_joint_reduction.cc
  joint_axis_frame.cc
  plugin_attribute.cc
  provide_feedback.cc
)

link_directories(${PROJECT_BINARY_DIR}/test)

sdf_build_tests(${tests})
