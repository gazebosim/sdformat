include (${project_cmake_dir}/SDFUtils.cmake)

# Generate a the ruby script.
# Note that the major version of the library is included in the name.
# Ex: cmdsdformat0.rb
if (APPLE)
  set(IGN_LIBRARY_NAME lib${PROJECT_NAME_LOWER}${SDF_MAJOR_VERSION}.dylib)
else()
  set(IGN_LIBRARY_NAME lib${PROJECT_NAME_LOWER}.so.${SDF_MAJOR_VERSION})
endif()

configure_file(
  "cmdsdformat.rb.in"
    "${CMAKE_CURRENT_BINARY_DIR}/cmdsdformat${SDF_MAJOR_VERSION}.rb" @ONLY)

# Install the ruby command line library in an unversioned location.
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/cmdsdformat${SDF_MAJOR_VERSION}.rb DESTINATION lib/ruby/ignition)
