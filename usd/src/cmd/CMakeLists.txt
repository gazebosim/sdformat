if(TARGET ${usd_target})
  add_executable(sdf2usd
    sdf2usd.cc
  )

  target_link_libraries(sdf2usd
    PUBLIC
      ignition-utils${IGN_UTILS_VER}::ignition-utils${IGN_UTILS_VER}
      ignition-utils${IGN_UTILS_VER}::cli
      ${usd_target}
  )

  add_executable(usd2sdf
    usd2sdf.cc
  )

  target_link_libraries(usd2sdf
    PUBLIC
      ignition-utils${IGN_UTILS_VER}::ignition-utils${IGN_UTILS_VER}
      ignition-utils${IGN_UTILS_VER}::cli
      ${usd_target}
  )

  install(
    TARGETS
    	sdf2usd
    	usd2sdf
    DESTINATION
    	${BIN_INSTALL_DIR}
  )
endif()
