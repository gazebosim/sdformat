# Build sdf CLI executable
set(sdf_executable gz-sdformat-sdf)
add_executable(${sdf_executable} sdf_main.cc gz.cc ../FrameSemantics.cc)
target_link_libraries(${sdf_executable}
  gz-utils::cli
  ${PROJECT_LIBRARY_TARGET_NAME}
  TINYXML2::TINYXML2
)
install(
  TARGETS ${sdf_executable}
  DESTINATION ${CMAKE_INSTALL_LIBEXECDIR}/gz/${GZ_DESIGNATION}${PROJECT_VERSION_MAJOR}/
)

# Build the unit tests only if Gazebo tools is installed
if(BUILD_TESTING AND GZ_PROGRAM)
  gz_build_tests(TYPE UNIT
    SOURCES
      gz_TEST.cc
    INCLUDE_DIRS
      ${PROJECT_SOURCE_DIR}/test
    TEST_LIST
      test_list
    LIB_DEPS
      gz-utils::gz-utils
      ${EXTRA_TEST_LIB_DEPS}
    ENVIRONMENT
      SDF_INSTALL_PREFIX=${CMAKE_INSTALL_PREFIX}
  )

  if(TARGET UNIT_gz_TEST)
    target_compile_definitions(UNIT_gz_TEST
      PRIVATE
        "GZ_PATH=\"${GZ_PROGRAM}\""
        "DETAIL_GZ_CONFIG_PATH=\"${CMAKE_BINARY_DIR}/test/conf/$<CONFIG>\""
        "GZ_TEST_LIBRARY_PATH=\"$<TARGET_FILE_DIR:${PROJECT_LIBRARY_TARGET_NAME}>\""
    )
  endif()
endif()

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
set(library_location "$<TARGET_FILE:${sdf_executable}>")

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

set(library_location "../../../${CMAKE_INSTALL_LIBEXECDIR}/gz/${GZ_DESIGNATION}${PROJECT_VERSION_MAJOR}/$<TARGET_FILE_NAME:${sdf_executable}>")

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
