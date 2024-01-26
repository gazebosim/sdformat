# Used only for internal testing.
set(gz_library_path "${CMAKE_BINARY_DIR}/test/lib/$<CONFIG>/ruby/gz/cmd${PROJECT_NAME}")

# Generate a configuration file for internal testing.
# Note that the major version of the library is included in the name.
# Ex: sdformat0.yaml
configure_file(
  "${PROJECT_NAME_NO_VERSION_LOWER}.yaml.in"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.yaml.configured" @ONLY)

file(GENERATE
  OUTPUT "${CMAKE_BINARY_DIR}/test/conf/$<CONFIG>/${PROJECT_NAME}.yaml"
  INPUT "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.yaml.configured")

# Used for the installed version.
set(gz_library_path "${CMAKE_INSTALL_PREFIX}/lib/ruby/gz/cmd${PROJECT_NAME}")

# Generate the configuration file that is installed.
# Note that the major version of the library is included in the name.
# Ex: sdformat0.yaml
configure_file(
  "${PROJECT_NAME_NO_VERSION_LOWER}.yaml.in"
  "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.yaml" @ONLY)

# Install the yaml configuration files in an unversioned location.
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.yaml
  DESTINATION ${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_DATAROOTDIR}/gz/)
