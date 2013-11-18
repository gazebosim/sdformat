include (${sdf_cmake_dir}/SDFUtils.cmake)

set (headers
  Assert.hh
  Console.hh
  Converter.hh
  Exception.hh
  Param.hh
  parser.hh
  SDFImpl.hh
  Types.hh
  system_util.hh
)

set (sdf_headers "" CACHE INTERNAL "SDF headers" FORCE)
foreach (hdr ${headers})
  APPEND_TO_CACHED_STRING(sdf_headers
    "SDF Headers" "#include <sdf/${hdr}>\n")
endforeach()
configure_file (${CMAKE_CURRENT_SOURCE_DIR}/sdf.hh.in
                ${CMAKE_CURRENT_BINARY_DIR}/sdf.hh)

sdf_install_includes("" ${headers} parser_urdf.hh
  ${CMAKE_CURRENT_BINARY_DIR}/sdf.hh)
