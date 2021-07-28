set(TEST_TYPE "INTEGRATION")

set(tests
  actor_dom.cc
  audio.cc
  collision_dom.cc
  default_elements.cc
  deprecated_specs.cc
  geometry_dom.cc
  include.cc
  joint_axis_dom.cc
  joint_dom.cc
  light_dom.cc
  link_dom.cc
  model_dom.cc
  model_versions.cc
  parser_error_detection.cc
  root_dom.cc
  scene_dom.cc
  sdf_basic.cc
  surface_dom.cc
  visual_dom.cc
  world_dom.cc
)

# tests which require Param.cc (e.g., uses Param::Get<T>)
set(param_required_tests
  category_bitmask.cc
  cfm_damping_implicit_spring_damper.cc
  converter.cc
  disable_fixed_joint_reduction.cc
  fixed_joint_reduction.cc
  force_torque_sensor.cc
  frame.cc
  includes.cc
  joint_axis_frame.cc
  link_light.cc
  locale_fix.cc
  locale_fix_cxx.cc
  material_pbr.cc
  nested_model.cc
  plugin_attribute.cc
  plugin_bool.cc
  plugin_include.cc
  provide_feedback.cc
  sdf_custom.cc
  unknown.cc
  urdf_gazebo_extensions.cc
  urdf_joint_parameters.cc
)

if (PYTHONINTERP_FOUND AND PY_PSUTIL)
  set(tests ${tests} element_memory_leak.cc)
endif()

find_program(XMLLINT_EXE xmllint)
if (EXISTS ${XMLLINT_EXE})
  set (tests ${tests} schema_test.cc)
else()
  BUILD_WARNING("xmllint not found. schema_test won't be run")
endif()

link_directories(${PROJECT_BINARY_DIR}/test)

sdf_build_tests(${tests})

if (NOT WIN32)
  set(SDF_BUILD_TESTS_EXTRA_EXE_SRCS ${CMAKE_SOURCE_DIR}/src/Param.cc)
  sdf_build_tests(${param_required_tests})
endif()

if (EXISTS ${XMLLINT_EXE})
  # Need to run schema target (build .xsd files) before running schema_test
  add_dependencies(${TEST_TYPE}_schema_test schema)
endif()

# Test symbols having the right name on linux only
if (UNIX AND NOT APPLE)
  configure_file(all_symbols_have_version.bash.in ${CMAKE_CURRENT_BINARY_DIR}/all_symbols_have_version.bash @ONLY)
  add_test(NAME INTEGRATION_versioned_symbols
    COMMAND bash ${CMAKE_CURRENT_BINARY_DIR}/all_symbols_have_version.bash ${CMAKE_BINARY_DIR}/src/libsdformat${SDF_MAJOR_VERSION}.so)
endif()
