# Used only for internal testing.
set(ign_library_path "${CMAKE_BINARY_DIR}/test/lib/ruby/ignition/cmd${PROJECT_NAME_LOWER}${SDF_MAJOR_VERSION}")

# Generate a configuration file for internal testing.
# Note that the major version of the library is included in the name.
# Ex: sdformat0.yaml
configure_file(
  "${PROJECT_NAME_LOWER}.yaml.in"
    "${CMAKE_BINARY_DIR}/test/conf/${PROJECT_NAME_LOWER}${SDF_MAJOR_VERSION}.yaml" @ONLY)

# Used for the installed version.
set(ign_library_path "${CMAKE_INSTALL_PREFIX}/lib/ruby/ignition/cmd${PROJECT_NAME_LOWER}${SDF_MAJOR_VERSION}")

# Generate the configuration file that is installed.
# Note that the major version of the library is included in the name.
# Ex: sdformat0.yaml
configure_file(
  "${PROJECT_NAME_LOWER}.yaml.in"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME_LOWER}${SDF_MAJOR_VERSION}.yaml" @ONLY)

# Install the yaml configuration files in an unversioned location.
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME_LOWER}${SDF_MAJOR_VERSION}.yaml
  DESTINATION ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/ignition/)
