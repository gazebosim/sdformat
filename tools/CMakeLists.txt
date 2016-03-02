include_directories(
  ${Boost_INCLUDE_DIR}
  ${IGNITION-MATH_INCLUDE_DIRS}
)

add_executable(sdf2urdf sdf2urdf.cc)
target_link_libraries(sdf2urdf sdformat)
sdf_install_executable(sdf2urdf)
