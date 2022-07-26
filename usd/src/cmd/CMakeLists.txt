if(TARGET ${usd_target})
  add_executable(sdf2usd
    sdf2usd.cc
    ../sdf_parser/Collision.cc
    ../sdf_parser/Model.cc
    ../sdf_parser/Joint.cc
    ../sdf_parser/Link.cc
    ../sdf_parser/Geometry.cc
    ../sdf_parser/Material.cc
    ../sdf_parser/Sensor.cc
    ../sdf_parser/Visual.cc
    ../sdf_parser/Light.cc
    ../Conversions.cc
  )

  target_link_libraries(sdf2usd
    PUBLIC
      gz-utils${GZ_UTILS_VER}::gz-utils${GZ_UTILS_VER}
      gz-utils${GZ_UTILS_VER}::cli
      ${usd_target}
  )

  add_executable(usd2sdf
    usd2sdf.cc
  )

  target_link_libraries(usd2sdf
    PUBLIC
      gz-utils${GZ_UTILS_VER}::gz-utils${GZ_UTILS_VER}
      gz-utils${GZ_UTILS_VER}::cli
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
