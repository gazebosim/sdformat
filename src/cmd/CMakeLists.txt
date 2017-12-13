include (${project_cmake_dir}/SDFUtils.cmake)

# Generate a the ruby script.
# Note that the major version of the library is included in the name.
# Ex: cmdsdformat0.rb
set(IGN_LIBRARY_NAME $<TARGET_FILE_NAME:sdformat>)
configure_file(
  "cmdsdformat.rb.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmdsdformat${SDF_MAJOR_VERSION}.rb" @ONLY)

# Install the ruby command line library in an unversioned location.
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/cmdsdformat${SDF_MAJOR_VERSION}.rb DESTINATION lib/ruby/ignition)
