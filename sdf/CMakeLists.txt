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

# Generate the EmbeddedSdf.cc file, which contains all the supported SDF
# descriptions in a map of strings. The parser.cc file uses EmbeddedSdf.hh.
set(EMBEDDED_SDF_CC_PATH "${PROJECT_BINARY_DIR}/src/EmbeddedSdf.cc")
execute_process(
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/sdf/embedSdf.py
    --output-file "${EMBEDDED_SDF_CC_PATH}"
  WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/sdf"
  RESULT_VARIABLE EMBEDDED_SDF_RESULT
  ERROR_VARIABLE EMBEDDED_SDF_ERROR
)
# check process return code
if(NOT EMBEDDED_SDF_RESULT EQUAL 0)
  message(FATAL_ERROR "Error executing ${Python3_EXECUTABLE} ${CMAKE_SOURCE_DIR}/sdf/embedSdf.py to create ${EMBEDDED_SDF_CC_PATH}: ${EMBEDDED_SDF_ERROR}")
endif()
# check that EmbeddedSdf.cc exists with non-zero size
# this should not happen if embedSdf.py was successful
if(EXISTS "${EMBEDDED_SDF_CC_PATH}")
  file(SIZE "${EMBEDDED_SDF_CC_PATH}" EMBEDDED_SDF_CC_SIZE)
endif()
if("${EMBEDDED_SDF_CC_SIZE}" STREQUAL "0" OR "${EMBEDDED_SDF_CC_SIZE}" STREQUAL "")
  message(FATAL_ERROR "${EMBEDDED_SDF_CC_PATH} is empty or does not exist")
endif()

# Generate aggregated SDF description files for use by the sdformat.org
# website. If the description files change, the generated full*.sdf files need
# to be removed before running this target.

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
      $<TARGET_FILE:gz-sdformat-sdf>
    ARGS sdf -d ${desc_ver} > ${CMAKE_CURRENT_BINARY_DIR}/full_${desc_ver_dash}.sdf
    COMMENT "Generating full description for spec ${desc_ver}"
    VERBATIM)
endforeach()
add_custom_target(sdf_descriptions DEPENDS ${description_targets})
# Add a dependency on the gz-sdformat-sdf target which is created in in ../cmd/CMakeLists
add_dependencies(sdf_descriptions gz-sdformat-sdf)
