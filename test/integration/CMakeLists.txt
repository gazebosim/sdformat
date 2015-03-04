set(TEST_TYPE "INTEGRATION")

set(tests
  audio.cc
  cfm_damping_implicit_spring_damper.cc
  disable_fixed_joint_reduction.cc
  fixed_joint_reduction.cc
  force_torque_sensor.cc
  joint_axis_frame.cc
  locale_fix.cc
  plugin_attribute.cc
  plugin_bool.cc
  provide_feedback.cc
  schema_test.cc
  urdf_joint_parameters.cc
)

link_directories(${PROJECT_BINARY_DIR}/test)

sdf_build_tests(${tests})

# Need to run schema target (build .xsd files) before running schema_test
add_dependencies(${TEST_TYPE}_schema_test schema)
