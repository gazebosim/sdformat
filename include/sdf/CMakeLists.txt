# Exlcude some files so that they are not added to the master header.
ign_install_all_headers(EXCLUDE_FILES sdf.hh sdf_config.h)
install(
  FILES
    sdf.hh
    sdf_config.h
  DESTINATION ${IGN_INCLUDE_INSTALL_DIR_FULL}/${PROJECT_INCLUDE_DIR}
)
