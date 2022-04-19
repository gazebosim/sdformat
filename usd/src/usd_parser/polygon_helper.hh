/*
 * Copyright (C) 2022 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef USD_PARSER_POLYGON_HELPER_HH
#define USD_PARSER_POLYGON_HELPER_HH

#include <vector>

#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/base/gf/vec3d.h>
#include <pxr/base/vt/array.h>
#pragma pop_macro ("__DEPRECATED")

#include "sdf/config.hh"
#include "sdf/usd/Export.hh"
#include "sdf/usd/UsdError.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  namespace usd
  {
  /// \brief Converts vertex indices of a polygon mesh to vertex indices
  /// of a triangle mesh.
  /// \details This uses the fan-triangulating algorithm, so it only works if
  /// all polygons in the mesh are convex.
  /// \param[in] _faceVertexIndices A flat list of vertex indices of a polygon
  /// mesh
  /// \param[in] _faceVertexCounts A list containing the number of vertices for
  /// each
  /// face of the mesh.
  /// \param[out] _triangles A flat list of vertex indices, with each face
  /// converted to one or more triangles.
  /// \return UsdErrors, which is a vector of UsdError objects. Each UsdError
  /// includes an error code and message. An empty vector indicates no error
  /// occurred when parsing the polygons.
  UsdErrors PolygonToTriangles(
    const pxr::VtIntArray &_faceVertexIndices,
    const pxr::VtIntArray &_faceVertexCounts,
    std::vector<unsigned int> &_triangles);
}
}
}

#endif
