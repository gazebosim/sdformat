#===============================================================================
# Generate the ruby script for internal testing.
# Note that the major version of the library is included in the name.
# Ex: cmdsdformat0.rb
set(cmd_script_generated_test "${CMAKE_BINARY_DIR}/test/lib/ruby/ignition/cmd${sdf_target}.rb")
set(cmd_script_configured_test "${cmd_script_generated_test}.configured")

# Set the library_location variable to the full path of the library file within
# the build directory.
set(library_location "$<TARGET_FILE:${sdf_target}>")

configure_file(
  "cmd${PROJECT_NAME_NO_VERSION_LOWER}.rb.in"
  "${cmd_script_configured_test}"
  @ONLY)

file(GENERATE
  OUTPUT "${cmd_script_generated_test}"
  INPUT  "${cmd_script_configured_test}")


#===============================================================================
# Used for the installed version.
# Generate the ruby script that gets installed.
# Note that the major version of the library is included in the name.
# Ex: cmdsdformat0.rb
set(cmd_script_generated "${CMAKE_CURRENT_BINARY_DIR}/cmd${sdf_target}.rb")
set(cmd_script_configured "${cmd_script_generated}.configured")

# Set the library_location variable to the relative path to the library file
# within the install directory structure.
set(library_location "../../../${CMAKE_INSTALL_LIBDIR}/$<TARGET_FILE_NAME:${sdf_target}>")

configure_file(
  "cmd${PROJECT_NAME_NO_VERSION_LOWER}.rb.in"
  "${cmd_script_configured}"
  @ONLY)

file(GENERATE
  OUTPUT "${cmd_script_generated}"
  INPUT  "${cmd_script_configured}")

# Install the ruby command line library in an unversioned location.
install(FILES ${cmd_script_generated} DESTINATION lib/ruby/ignition)

#===============================================================================
# SDF -> USD converter
find_package(ignition-common4 COMPONENTS graphics REQUIRED)
set(IGN_COMMON_VER ${ignition-common4_VERSION_MAJOR})
add_executable(usdConverter
  usdConverter.cc
)

target_link_libraries(usdConverter
  PUBLIC
    ignition-common${IGN_COMMON_VER}::graphics
    ${sdf_target}
)

install(
  TARGETS
    usdConverter
  DESTINATION
    ${BIN_INSTALL_DIR}
)
