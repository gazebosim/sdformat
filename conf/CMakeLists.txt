include (${project_cmake_dir}/SDFUtils.cmake)

set(ign_library_path "${CMAKE_BINARY_DIR}/src/cmd/cmdsdformat${SDF_MAJOR_VERSION}")

# Generate a configuration file for internal testing.
# Note that the major version of the library is included in the name.
# Ex: sdformat0.yaml
configure_file(
  "sdformat.yaml.in"
    "${CMAKE_BINARY_DIR}/test/conf/sdformat${SDF_MAJOR_VERSION}.yaml" @ONLY)

set(ign_library_path "${CMAKE_INSTALL_PREFIX}/lib/ruby/ignition/cmdsdformat${SDF_MAJOR_VERSION}")

# Generate a configuration file.
# Note that the major version of the library is included in the name.
# Ex: sdformat0.yaml
configure_file(
  "sdformat.yaml.in"
    "${CMAKE_CURRENT_BINARY_DIR}/sdformat${SDF_MAJOR_VERSION}.yaml" @ONLY)

# Install the yaml configuration files in an unversioned location.
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/sdformat${SDF_MAJOR_VERSION}.yaml DESTINATION ${CMAKE_INSTALL_FULL_DATAROOTDIR}/ignition/)
