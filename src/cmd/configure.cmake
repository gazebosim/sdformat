message(STATUS "Generating ruby script ${OUTPUT_FILE}")
configure_file("${INPUT_FILE}" "${OUTPUT_FILE}" @ONLY)
