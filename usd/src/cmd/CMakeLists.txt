if(TARGET ${usd_target})
  add_executable(sdf2usd
    sdf2usd.cc
  )

  target_link_libraries(sdf2usd
    PUBLIC
      ignition-utils${IGN_UTILS_VER}::ignition-utils${IGN_UTILS_VER}
      ignition-common${IGN_COMMON_VER}::ignition-common${IGN_COMMON_VER}
      ignition-common${IGN_COMMON_VER}::graphics
      ${usd_target}
  )

  install(
    TARGETS
    	sdf2usd
    DESTINATION
    	${BIN_INSTALL_DIR}
  )
endif()
