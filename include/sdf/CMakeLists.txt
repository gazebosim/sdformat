include (${sdf_cmake_dir}/SDFUtils.cmake)

set (headers
  Console.hh
  Converter.hh
  Param.hh
  parser.hh
  SDF.hh
  Types.hh
)

if (HAVE_URDFDOM AND HAVE_URDFDOM_HEADERS)
  set (headers ${headers} parser_urdf.hh)
endif(HAVE_URDFDOM AND HAVE_URDFDOM_HEADERS)

set (sdf_headers "" CACHE INTERNAL "SDF headers" FORCE)
foreach (hdr ${headers})
  APPEND_TO_CACHED_STRING(sdf_headers
    "SDF Headers" "#include <sdf/${hdr}>\n")
endforeach()
configure_file (${CMAKE_CURRENT_SOURCE_DIR}/sdf.hh.in
                ${CMAKE_CURRENT_BINARY_DIR}/sdf.hh)

sdf_install_includes("" ${headers} ${CMAKE_CURRENT_BINARY_DIR}/sdf.hh)
