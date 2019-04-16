add_subdirectory(1.0)
add_subdirectory(1.2)
add_subdirectory(1.3)
add_subdirectory(1.4)
add_subdirectory(1.5)
add_subdirectory(1.6)

add_custom_target(schema)
add_dependencies(schema schema1_6)

# Generate the EmbeddedSdf.hh file, which contains all the supported SDF
# descriptions in a map of strings. The parser.cc file uses EmbeddedSdf.hh.
execute_process(
  COMMAND ${RUBY} ${CMAKE_SOURCE_DIR}/sdf/embedSdf.rb
  WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/sdf"
  OUTPUT_FILE "${PROJECT_BINARY_DIR}/include/sdf/EmbeddedSdf.hh"
)


