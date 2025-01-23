add_subdirectory(1.0)
add_subdirectory(1.2)
add_subdirectory(1.3)
add_subdirectory(1.4)
add_subdirectory(1.5)
add_subdirectory(1.6)
add_subdirectory(1.7)
add_subdirectory(1.8)
add_subdirectory(1.9)
add_subdirectory(1.10)
add_subdirectory(1.11)
add_subdirectory(1.12)

add_custom_target(schema)
add_dependencies(schema schema1_11)

if (NOT Python3_Interpreter_FOUND)
  gz_build_error("Python is required to generate the C++ file with the SDF content")
endif()

# Generate the EmbeddedSdf.cc file, which contains all the supported SDF
# descriptions in a map of strings. The parser.cc file uses EmbeddedSdf.hh.
execute_process(
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/sdf/embedSdf.py
    --output-file "${PROJECT_BINARY_DIR}/src/EmbeddedSdf.cc"
  WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/sdf"
)

# Generate aggregated SDF description files for use by the sdformat.org
# website. If the description files change, the generated full*.sdf files need
# to be removed before running this target.
if (GZ_PROGRAM)

  # Update this list as new sdformat spec versions are added.
  set(sdf_desc_versions 1.4 1.5 1.6 1.7 1.8 1.9 1.10 1.11 1.12)

  set(description_targets)
  foreach(desc_ver ${sdf_desc_versions})
    string(REPLACE "." "-" desc_ver_dash ${desc_ver})
    list(APPEND description_targets ${CMAKE_CURRENT_BINARY_DIR}/full_${desc_ver_dash}.sdf)
    add_custom_command(
      OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/full_${desc_ver_dash}.sdf
      COMMAND
        ${CMAKE_COMMAND} -E env GZ_CONFIG_PATH=${CMAKE_BINARY_DIR}/test/conf/$<CONFIG>
        ${GZ_PROGRAM}
      ARGS sdf -d ${desc_ver} > ${CMAKE_CURRENT_BINARY_DIR}/full_${desc_ver_dash}.sdf
      COMMENT "Generating full description for spec ${desc_ver}"
      VERBATIM)
  endforeach()
  add_custom_target(sdf_descriptions DEPENDS ${description_targets})
  # Add a dependency on the gz-sdformat-sdf target which is created in in ../cmd/CMakeLists
  add_dependencies(sdf_descriptions gz-sdformat-sdf)
endif()
