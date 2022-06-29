if(WIN32 AND CMAKE_BUILD_TYPE STREQUAL "Debug")
  # pybind11 logic for setting up a debug build when both a debug and release
  # python interpreter are present in the system seems to be pretty much broken.
  # This works around the issue.
  set(PYTHON_LIBRARIES "${PYTHON_DEBUG_LIBRARIES}")
endif()


if(USE_SYSTEM_PATHS_FOR_PYTHON_INSTALLATION)
  if(${CMAKE_VERSION} VERSION_LESS "3.12.0")
    execute_process(
      COMMAND "${PYTHON_EXECUTABLE}" -c "if True:
  from distutils import sysconfig as sc
  print(sc.get_python_lib(plat_specific=True))"
      OUTPUT_VARIABLE Python3_SITEARCH
      OUTPUT_STRIP_TRAILING_WHITESPACE)
  else()
    # Get install variable from Python3 module
    # Python3_SITEARCH is available from 3.12 on, workaround if needed:
    find_package(Python3 COMPONENTS Interpreter)
  endif()

  if(USE_DIST_PACKAGES_FOR_PYTHON)
    string(REPLACE "site-packages" "dist-packages" IGN_PYTHON_INSTALL_PATH ${Python3_SITEARCH})
  endif()
else()
  # If not a system installation, respect local paths
  set(IGN_PYTHON_INSTALL_PATH ${IGN_LIB_INSTALL_DIR}/python)
endif()

# Set the build location and install location for a CPython extension
function(configure_build_install_location _library_name)
  # Install library for actual use
  install(TARGETS ${_library_name}
    DESTINATION "${IGN_PYTHON_INSTALL_PATH}"
  )
endfunction()

pybind11_add_module(sdformat SHARED
  src/sdf/_ignition_sdformat_pybind11.cc
  src/sdf/pyAirPressure.cc
  src/sdf/pyAltimeter.cc
  src/sdf/pyAtmosphere.cc
  src/sdf/pyBox.cc
  src/sdf/pyCamera.cc
  src/sdf/pyCapsule.cc
  src/sdf/pyCollision.cc
  src/sdf/pyCylinder.cc
  src/sdf/pyEllipsoid.cc
  src/sdf/pyError.cc
  src/sdf/pyForceTorque.cc
  src/sdf/pyFrame.cc
  src/sdf/pyGeometry.cc
  src/sdf/pyHeightmap.cc
  src/sdf/pyIMU.cc
  src/sdf/pyJoint.cc
  src/sdf/pyJointAxis.cc
  src/sdf/pyLidar.cc
  src/sdf/pyLight.cc
  src/sdf/pyLink.cc
  src/sdf/pyMagnetometer.cc
  src/sdf/pyMaterial.cc
  src/sdf/pyMesh.cc
  src/sdf/pyModel.cc
  src/sdf/pyNavSat.cc
  src/sdf/pyNoise.cc
  src/sdf/pyParserConfig.cc
  src/sdf/pyParticleEmitter.cc
  src/sdf/pyPbr.cc
  src/sdf/pyPlane.cc
  src/sdf/pyPlugin.cc
  src/sdf/pyRoot.cc
  src/sdf/pyScene.cc
  src/sdf/pySemanticPose.cc
  src/sdf/pySensor.cc
  src/sdf/pySky.cc
  src/sdf/pySphere.cc
  src/sdf/pySurface.cc
  src/sdf/pyVisual.cc
  src/sdf/pyWorld.cc
  src/sdf/pybind11_helpers.cc
)

target_link_libraries(sdformat PRIVATE
  ${PROJECT_LIBRARY_TARGET_NAME}
)

configure_build_install_location(sdformat)

if (BUILD_TESTING)
  pybind11_add_module(sdformattest SHARED
    test/_ignition_sdformattest_pybind11.cc
  )

  target_link_libraries(sdformattest PRIVATE
    ${PROJECT_LIBRARY_TARGET_NAME}
    gz-utils${GZ_UTILS_VER}::gz-utils${GZ_UTILS_VER}
  )

  set(python_tests
    pyAirPressure_TEST
    pyAltimeter_TEST
    pyAtmosphere_TEST
    pyBox_TEST
    pyCamera_TEST
    pyCapsule_TEST
    pyCollision_TEST
    pyCylinder_TEST
    pyEllipsoid_TEST
    pyError_TEST
    pyForceTorque_TEST
    pyFrame_TEST
    pyGeometry_TEST
    pyHeightmap_TEST
    pyIMU_TEST
    pyJoint_TEST
    pyJointAxis_TEST
    pyLidar_TEST
    pyLight_TEST
    pyLink_TEST
    pyMagnetometer_TEST
    pyMaterial_TEST
    pyMesh_TEST
    pyModel_TEST
    pyNoise_TEST
    pyNavSat_TEST
    pyParserConfig_TEST
    pyParticleEmitter_TEST
    pyPbr_TEST
    pyPlane_TEST
    pyPlugin_TEST
    pyRoot_TEST
    pyScene_TEST
    pySemanticPose_TEST
    pySensor_TEST
    pySky_TEST
    pySphere_TEST
    pySurface_TEST
    pyVisual_TEST
    pyWorld_TEST
  )
  execute_process(COMMAND "${Python3_EXECUTABLE}" -m pytest --version
    OUTPUT_VARIABLE PYTEST_output
    ERROR_VARIABLE  PYTEST_error
    RESULT_VARIABLE PYTEST_result)
  if(${PYTEST_result} EQUAL 0)
    set(pytest_FOUND TRUE)
  else()
    message("")
    message(WARNING "Pytest package not available: ${PYTEST_error}")
  endif()

  foreach (test ${python_tests})
    if (pytest_FOUND)
      add_test(NAME ${test}.py COMMAND
        "${Python3_EXECUTABLE}" -m pytest "${CMAKE_SOURCE_DIR}/python/test/${test}.py" --junitxml "${CMAKE_BINARY_DIR}/test_results/${test}.xml")
    else()
      add_test(NAME ${test}.py COMMAND
        "${Python3_EXECUTABLE}" "${CMAKE_SOURCE_DIR}/python/test/${test}.py")
    endif()
    set(_env_vars)
    list(APPEND _env_vars "PYTHONPATH=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}/python/:${CMAKE_BINARY_DIR}/lib:$ENV{PYTHONPATH}")
    list(APPEND _env_vars "LD_LIBRARY_PATH=${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}:$ENV{LD_LIBRARY_PATH}")
    set_tests_properties(${test}.py PROPERTIES
      ENVIRONMENT "${_env_vars}")
  endforeach()
endif()
