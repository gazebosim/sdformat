#===============================================================================
# Generate the ruby script for internal testing.
# Note that the major version of the library is included in the name.
# Ex: cmdsdformat0.rb
set(cmd_script_generated_test 
  "${CMAKE_BINARY_DIR}/test/lib/$<CONFIG>/ruby/gz/cmd${PROJECT_NAME}.rb")
set(cmd_script_configured_test 
  "${CMAKE_CURRENT_BINARY_DIR}/test_cmd${PROJECT_NAME}.rb.configured")

# Set the library_location variable to the full path of the library file within
# the build directory.
set(library_location "$<TARGET_FILE:${PROJECT_NAME}>")

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
set(cmd_script_generated "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/cmd${PROJECT_NAME}.rb")
set(cmd_script_configured "${CMAKE_CURRENT_BINARY_DIR}/cmd${PROJECT_NAME}.rb.configured")

# Set the library_location variable to the relative path to the library file
# within the install directory structure.
if (MSVC)
  set(library_location_prefix "${CMAKE_INSTALL_BINDIR}")
else()
  set(library_location_prefix "${CMAKE_INSTALL_LIBDIR}")
endif()

set(library_location "../../../${library_location_prefix}/$<TARGET_FILE_NAME:${PROJECT_NAME}>")

configure_file(
  "cmd${PROJECT_NAME_NO_VERSION_LOWER}.rb.in"
  "${cmd_script_configured}"
  @ONLY)

file(GENERATE
  OUTPUT "${cmd_script_generated}"
  INPUT  "${cmd_script_configured}")

# Install the ruby command line library in an unversioned location.
install(FILES ${cmd_script_generated} DESTINATION lib/ruby/gz)

#===============================================================================
# Bash completion

# Tack version onto and install the bash completion script
configure_file(
  "sdf.bash_completion.sh"
    "${CMAKE_CURRENT_BINARY_DIR}/sdf${PROJECT_VERSION_MAJOR}.bash_completion.sh" @ONLY)
install(
  FILES
    ${CMAKE_CURRENT_BINARY_DIR}/sdf${PROJECT_VERSION_MAJOR}.bash_completion.sh
  DESTINATION
    ${CMAKE_INSTALL_DATAROOTDIR}/gz/gz${GZ_TOOLS_VER}.completion.d)
