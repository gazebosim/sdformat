configure_file(config.hh.in
  include/sdf/config.hh)

ign_install_all_headers(EXCLUDE_FILES sdf.hh sdf_config.h)
