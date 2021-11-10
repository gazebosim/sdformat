include (${sdf_cmake_dir}/SDFUtils.cmake)

set (sources
  Actor.cc
  AirPressure.cc
  Altimeter.cc
  Atmosphere.cc
  Box.cc
  Camera.cc
  Capsule.cc
  Collision.cc
  Console.cc
  Converter.cc
  Cylinder.cc
  Element.cc
  Ellipsoid.cc
  EmbeddedSdf.cc
  Error.cc
  Exception.cc
  Frame.cc
  FrameSemantics.cc
  Filesystem.cc
  ForceTorque.cc
  Geometry.cc
  Gui.cc
  Heightmap.cc
  ign.cc
  InterfaceElements.cc
  InterfaceFrame.cc
  InterfaceJoint.cc
  InterfaceLink.cc
  InterfaceModel.cc
  InterfaceModelPoseGraph.cc
  Imu.cc
  Joint.cc
  JointAxis.cc
  Lidar.cc
  Light.cc
  Link.cc
  Magnetometer.cc
  Material.cc
  Mesh.cc
  Model.cc
  NavSat.cc
  Noise.cc
  parser.cc
  parser_urdf.cc
  ParserConfig.cc
  Param.cc
  ParamPassing.cc
  ParticleEmitter.cc
  Pbr.cc
  Physics.cc
  Plane.cc
  PrintConfig.cc
  Root.cc
  Scene.cc
  SDF.cc
  SDFExtension.cc
  SemanticPose.cc
  Sensor.cc
  Sky.cc
  Sphere.cc
  Surface.cc
  Types.cc
  Utils.cc
  Visual.cc
  World.cc
  XmlUtils.cc
)
include_directories(${CMAKE_CURRENT_SOURCE_DIR})

if (USE_INTERNAL_URDF)
  set(sources ${sources}
  urdf/urdf_parser/model.cpp
  urdf/urdf_parser/link.cpp
  urdf/urdf_parser/joint.cpp
  urdf/urdf_parser/pose.cpp
  urdf/urdf_parser/twist.cpp
  urdf/urdf_parser/urdf_model_state.cpp
  urdf/urdf_parser/urdf_sensor.cpp
  urdf/urdf_parser/world.cpp)
endif()

if (BUILD_SDF_TEST)
  set (gtest_sources
    Actor_TEST.cc
    AirPressure_TEST.cc
    Altimeter_TEST.cc
    Atmosphere_TEST.cc
    Box_TEST.cc
    Camera_TEST.cc
    Capsule_TEST.cc
    Collision_TEST.cc
    Console_TEST.cc
    Cylinder_TEST.cc
    Element_TEST.cc
    Ellipsoid_TEST.cc
    Error_TEST.cc
    Exception_TEST.cc
    Frame_TEST.cc
    Filesystem_TEST.cc
    ForceTorque_TEST.cc
    Geometry_TEST.cc
    Gui_TEST.cc
    Heightmap_TEST.cc
    Imu_TEST.cc
    InterfaceElements_TEST.cc
    Joint_TEST.cc
    JointAxis_TEST.cc
    Lidar_TEST.cc
    Light_TEST.cc
    Link_TEST.cc
    Magnetometer_TEST.cc
    Material_TEST.cc
    Mesh_TEST.cc
    Model_TEST.cc
    NavSat_TEST.cc
    Noise_TEST.cc
    Param_TEST.cc
    parser_TEST.cc
    ParserConfig_TEST.cc
    ParticleEmitter_TEST.cc
    Pbr_TEST.cc
    Physics_TEST.cc
    Plane_TEST.cc
    Root_TEST.cc
    Scene_TEST.cc
    SemanticPose_TEST.cc
    SDF_TEST.cc
    Sensor_TEST.cc
    Sky_TEST.cc
    Sphere_TEST.cc
    Surface_TEST.cc
    Types_TEST.cc
    Visual_TEST.cc
    World_TEST.cc
  )

  # Build this test file only if Ignition Tools is installed.
  if (IGNITION-TOOLS_BINARY_DIRS)
    set (gtest_sources ${gtest_sources}
      ign_TEST.cc
    )
  endif()

  if (NOT WIN32)
    # set(SDF_BUILD_TESTS_EXTRA_EXE_SRCS Utils.cc)
    set(gtest_sources ${gtest_sources} Utils_TEST.cc)
  endif()

  if (NOT WIN32)
    # set(SDF_BUILD_TESTS_EXTRA_EXE_SRCS XmlUtils.cc)
    set(gtest_sources ${gtest_sources} XmlUtils_TEST.cc)
  endif()

  if (NOT WIN32)
    # set(SDF_BUILD_TESTS_EXTRA_EXE_SRCS FrameSemantics.cc)
    set(gtest_sources ${gtest_sources} FrameSemantics_TEST.cc)
  endif()

  if (NOT WIN32)
    # set(SDF_BUILD_TESTS_EXTRA_EXE_SRCS Converter.cc EmbeddedSdf.cc XmlUtils.cc)
    set(gtest_sources ${gtest_sources} Converter_TEST.cc)
  endif()

  if (NOT WIN32)
    # set(SDF_BUILD_TESTS_EXTRA_EXE_SRCS SDFExtension.cc parser_urdf.cc XmlUtils.cc)
    set(gtest_sources ${gtest_sources} parser_urdf_TEST.cc)
  endif()

  if (NOT WIN32)
    # set(SDF_BUILD_TESTS_EXTRA_EXE_SRCS ParamPassing.cc XmlUtils.cc parser.cc
    # parser_urdf.cc FrameSemantics.cc Converter.cc EmbeddedSdf.cc SDFExtension.cc Utils.cc)
    set(gtest_sources ${gtest_sources} ParamPassing_TEST.cc)
  endif()

  ign_build_tests(
    TYPE UNIT
    SOURCES ${gtest_sources}
    # FIXME: This adds dependencies to ALL tests.
    INCLUDE_DIRS ${PROJECT_SOURCE_DIR}/test
  )

  if (TARGET UNIT_ign_TEST)
    target_link_libraries(UNIT_ign_TEST ignition-cmake${IGN_CMAKE_VER}::utilities)
  endif()

  if (TARGET UNIT_XmlUtils_TEST)
    target_link_libraries(UNIT_XmlUtils_TEST ${TINYXML2_LIBRARIES})
  endif()

  if (TARGET UNIT_Converter_TEST)
    target_link_libraries(UNIT_Converter_TEST ${TINYXML2_LIBRARIES})
  endif()

  if (TARGET UNIT_parser_urdf_TEST)
    if (NOT USE_INTERNAL_URDF)
      target_compile_options(UNIT_parser_urdf_TEST PRIVATE ${URDF_CFLAGS})
      if (${CMAKE_VERSION} VERSION_GREATER 3.13)
        target_link_options(UNIT_parser_urdf_TEST PRIVATE ${URDF_LDFLAGS})
      endif()
      target_link_libraries(UNIT_parser_urdf_TEST ${URDF_LIBRARIES})
    endif()
    target_link_libraries(UNIT_parser_urdf_TEST ${TINYXML2_LIBRARIES})
  endif()

  if (TARGET UNIT_ParamPassing_TEST)
    if (NOT USE_INTERNAL_URDF)
      target_compile_options(UNIT_ParamPassing_TEST PRIVATE ${URDF_CFLAGS})
      if (${CMAKE_VERSION} VERSION_GREATER 3.13)
        target_link_options(UNIT_ParamPassing_TEST PRIVATE ${URDF_LDFLAGS})
      endif()
      target_link_libraries(UNIT_ParamPassing_TEST ${URDF_LIBRARIES})
    endif()
    target_link_libraries(UNIT_ParamPassing_TEST ${TINYXML2_LIBRARIES})
  endif()
endif()

ign_create_core_library(SOURCES ${sources} CXX_STANDARD 17)

target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}
  PUBLIC
    ignition-math${IGN_MATH_VER}::ignition-math${IGN_MATH_VER}
    ignition-utils${IGN_UTILS_VER}::ignition-utils${IGN_UTILS_VER}
  PRIVATE
    ${TINYXML2_LIBRARIES}
)

if (USE_INTERNAL_URDF)
  target_include_directories(${PROJECT_LIBRARY_TARGET_NAME}
    PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/urdf
  )
else()
  target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME}
    PRIVATE
      ${URDF_LIBRARIES}
  )
endif()

if (WIN32)
  target_compile_definitions(${PROJECT_LIBRARY_TARGET_NAME} PRIVATE URDFDOM_STATIC)
endif()

target_include_directories(${PROJECT_LIBRARY_TARGET_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}/..>
)

message (STATUS "URDF_LIBRARY_DIRS=${URDF_LIBRARY_DIRS}")
message (STATUS "URDF_LIBRARIES=${URDF_LIBRARIES}")

if (NOT USE_INTERNAL_URDF)
  target_compile_options(${PROJECT_LIBRARY_TARGET_NAME} PRIVATE ${URDF_CFLAGS})
  if (${CMAKE_VERSION} VERSION_GREATER 3.13)
    target_link_options(${PROJECT_LIBRARY_TARGET_NAME} PRIVATE ${URDF_LDFLAGS})
  endif()
  target_link_libraries(${PROJECT_LIBRARY_TARGET_NAME} PRIVATE ${URDF_LIBRARIES})
endif()

if(NOT WIN32)
  add_subdirectory(cmd)
endif()
