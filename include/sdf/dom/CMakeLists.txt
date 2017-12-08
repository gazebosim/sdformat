include (${sdf_cmake_dir}/SDFUtils.cmake)

set(headers
  Root.hh
)

sdf_install_includes("dom" ${headers})
