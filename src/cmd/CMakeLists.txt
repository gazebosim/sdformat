include (${project_cmake_dir}/SDFUtils.cmake)

# Generate a the ruby script.
# Note that the major version of the library is included in the name.
# Ex: cmdsdformat0.rb

get_filename_component(input_script "${CMAKE_CURRENT_SOURCE_DIR}/cmdsdformat.rb.in" ABSOLUTE)
get_filename_component(output_script "${CMAKE_CURRENT_BINARY_DIR}/cmdsdformat${SDF_MAJOR_VERSION}.rb" ABSOLUTE)

add_custom_command (
  OUTPUT ${output_script}
  COMMAND ${CMAKE_COMMAND}
    "-DINPUT_FILE=${input_script}"
    "-DOUTPUT_FILE=${output_script}"
    "-DIGN_LIBRARY_NAME=$<TARGET_FILE_NAME:sdformat>"
    "-DSDF_VERSION_FULL=${SDF_VERSION_FULL}"
    -P "${CMAKE_CURRENT_SOURCE_DIR}/configure.cmake"
)

add_custom_target(generate_ruby_script ALL DEPENDS ${output_script})

# Install the ruby command line library in an unversioned location.
install(FILES ${output_script} DESTINATION lib/ruby/ignition)
