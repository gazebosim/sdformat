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

#include "polygon_helper.hh"

#include <vector>
#include <cassert>

namespace sdf
{
// Inline bracke to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
//
namespace usd
{
  UsdErrors PolygonToTriangles(
      const pxr::VtIntArray &_faceVertexIndices,
      const pxr::VtIntArray &_faceVertexCounts,
      std::vector<unsigned int> &_triangles)
  {
    UsdErrors errors;
    // TODO(koon peng) Use more robust algorithms.
    // For reference, blender supports "ear-clipping", and "Beauty".
    // https://blender.stackexchange.com/questions/215553/what-algorithm-is-used-for-beauty-triangulation
    // ref: https://en.wikipedia.org/wiki/Polygon_triangulation
    size_t count = 0;
    for (const auto &vCount : _faceVertexCounts)
    {
      count += vCount - 2;
    }
    _triangles.reserve(count * 3);

    size_t cur = 0;
    for (const auto &vCount : _faceVertexCounts)
    {
      for (size_t i = cur + 2; i < cur + vCount; i++)
      {
        _triangles.emplace_back(_faceVertexIndices[cur]);
        _triangles.emplace_back(_faceVertexIndices[i - 1]);
        _triangles.emplace_back(_faceVertexIndices[i]);
      }
      cur += vCount;
    }

    if (_triangles.size() != count * 3)
    {
      errors.push_back(UsdError(UsdErrorCode::USD_TO_SDF_POLYGON_PARSING_ERROR,
            "Unable to parse the polugon mesh"));
      return errors;
    }

    return errors;
  }
}
}
}
