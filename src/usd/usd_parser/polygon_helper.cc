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

#include "polygon_helper.hh"

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Delaunay_triangulation_3.h>
#include <CGAL/Triangulation_vertex_base_with_info_3.h>
#include <CGAL/Random.h>
#include <vector>
#include <cassert>

typedef CGAL::Exact_predicates_inexact_constructions_kernel          K;
typedef CGAL::Triangulation_vertex_base_with_info_3<unsigned int, K> Vb;
typedef CGAL::Delaunay_triangulation_cell_base_3<K>                  Cb;
typedef CGAL::Triangulation_data_structure_3<Vb, Cb>                 Tds;
typedef CGAL::Delaunay_triangulation_3<K, Tds>                       Delaunay;
typedef Delaunay::Point                                              Point;

namespace usd
{
  std::vector<unsigned int> PolygonToTriangles(
    pxr::VtIntArray &_faceVertexIndices,
    pxr::VtIntArray &_faceVertexCounts,
    pxr::VtArray<pxr::GfVec3f> &_points)
  {
    std::vector<unsigned int> indices;

    unsigned int indexVertex = 0;
    for (unsigned int i = 0; i < _faceVertexCounts.size(); ++i)
    {
      if (_faceVertexCounts[i] == 3)
      {
        unsigned int j = indexVertex;
        unsigned int indexVertexStop = indexVertex + 3;
        for (; j < indexVertexStop; ++j)
        {
          indices.emplace_back(_faceVertexIndices[j]);
          ++indexVertex;
        }
      }
      else
      {
        std::vector< std::pair<Point, unsigned> > P;
        for (int j = 0; j < _faceVertexCounts[i]; ++j)
        {
          pxr::GfVec3f & _p = _points[_faceVertexIndices[indexVertex]];
          P.push_back(
            std::make_pair(
              Point(_p[0], _p[1], _p[2]),
              _faceVertexIndices[indexVertex]));
          ++indexVertex;
        }
        Delaunay triangulation(P.begin(), P.end());

        bool goodTriangle = true;
        for(Delaunay::Finite_facets_iterator fit = triangulation.finite_facets_begin();
            fit != triangulation.finite_facets_end(); ++fit)
        {
          auto &face = fit;
          goodTriangle = false;
        }

        if (goodTriangle)
        {
          for(Delaunay::Finite_facets_iterator fit = triangulation.finite_facets_begin();
              fit != triangulation.finite_facets_end(); ++fit)
          {
            auto &face = fit;
            indices.emplace_back(face->first->vertex(0)->info());
            indices.emplace_back(face->first->vertex(1)->info());
            indices.emplace_back(face->first->vertex(2)->info());
          }
        }
      }
    }
    return indices;
  }
}
