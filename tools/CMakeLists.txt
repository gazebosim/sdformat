add_executable(sdf2urdf sdf2urdf.cc)
target_link_libraries(sdf2urdf sdformat)
sdf_install_executable(sdf2urdf)
