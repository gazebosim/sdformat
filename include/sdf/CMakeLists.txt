include (${sdf_cmake_dir}/SDFUtils.cmake)

set (headers
  Console.hh
  Converter.hh
  Param.hh
  parser.hh
  SDF.hh
)

if (HAVE_URDFDOM AND HAVE_URDFDOM_HEADERS)
  set (headers ${headers} parser_urdf.hh)
endif(HAVE_URDFDOM AND HAVE_URDFDOM_HEADERS)

sdf_install_includes("" ${headers})
