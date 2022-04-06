/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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

namespace sdf
{
  // Inline bracke to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  namespace usd
  {
  ///\brief Converts a vertex indicies of a polygon mesh to a vertex indicies of a triangle mesh.
  ///\details This uses the fan-triangulating algorithm, so it only works if all polygons in
  /// in the mesh are convex.
  ///\param _faceVertexIndices A flat list of vertex indicies of a polygon mesh.
  ///\param _faceVertexCounts A list containing the number of vertices for each face of the mesh.
  ///\return A flat list of vertex indicies, with each face converted to one or more triangles.
  std::vector<unsigned int> PolygonToTriangles(
    pxr::VtIntArray &_faceVertexIndices,
    pxr::VtIntArray &_faceVertexCounts);
}
}
}

#endif
