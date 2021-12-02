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

#include "sdf_usd_parser/geometry.hh"

#include <iostream>
#include <string>

#include <pxr/base/gf/vec3f.h>
#include <pxr/base/vt/array.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/cube.h>
#include <pxr/usd/usdGeom/xformCommonAPI.h>

#include "sdf/Geometry.hh"
#include "sdf/Box.hh"

namespace usd
{
  bool ParseSdfBoxGeometry(const sdf::Geometry &_geometry, pxr::UsdStageRefPtr &_stage,
      const std::string &_path)
  {
    // USD defines a cube (i.e., a box with all dimensions being equal),
    // but not a box. So, we will take a 1x1x1 cube and scale it according
    // to the SDF box's dimensions to achieve varying dimensions
    auto usdCube = pxr::UsdGeomCube::Define(_stage, pxr::SdfPath(_path));
    usdCube.CreateSizeAttr().Set(1.0);
    pxr::GfVec3f endPoint(0.5);
    pxr::VtArray<pxr::GfVec3f> extentBounds;
    extentBounds.push_back(-1.0 * endPoint);
    extentBounds.push_back(endPoint);
    usdCube.CreateExtentAttr().Set(extentBounds);

    pxr::UsdGeomXformCommonAPI cubeXformAPI(usdCube);
    cubeXformAPI.SetScale(pxr::GfVec3f(
          _geometry.BoxShape()->Size().X(),
          _geometry.BoxShape()->Size().Y(),
          _geometry.BoxShape()->Size().Z()));
    return true;
  }

  bool ParseSdfCylinderGeometry(const sdf::Geometry &_geometry, pxr::UsdStageRefPtr &_stage,
      const std::string &_path)
  {
    // TODO(adlarkin) finish this
    return false;
  }

  bool ParseSdfSphereGeometry(const sdf::Geometry &_geometry, pxr::UsdStageRefPtr &_stage,
      const std::string &_path)
  {
    // TODO(adlarkin) finish this
    return false;
  }

  bool ParseSdfMeshGeometry(const sdf::Geometry &_geometry, pxr::UsdStageRefPtr &_stage,
      const std::string &_path)
  {
    // TODO(adlarkin) finish this
    return false;
  }

  bool ParseSdfCapsuleGeometry(const sdf::Geometry &_geometry, pxr::UsdStageRefPtr &_stage,
      const std::string &_path)
  {
    // TODO(adlarkin) finish this
    return false;
  }

  bool ParseSdfGeometry(const sdf::Geometry &_geometry, pxr::UsdStageRefPtr &_stage,
      const std::string &_path)
  {
    bool typeParsed = false;
    switch (_geometry.Type())
    {
      case sdf::GeometryType::BOX:
        typeParsed = ParseSdfBoxGeometry(_geometry, _stage, _path);
        break;
      case sdf::GeometryType::CYLINDER:
        typeParsed = ParseSdfCylinderGeometry(_geometry, _stage, _path);
        break;
      case sdf::GeometryType::SPHERE:
        typeParsed = ParseSdfSphereGeometry(_geometry, _stage, _path);
        break;
      case sdf::GeometryType::MESH:
        typeParsed = ParseSdfMeshGeometry(_geometry, _stage, _path);
        break;
      case sdf::GeometryType::CAPSULE:
        typeParsed = ParseSdfCapsuleGeometry(_geometry, _stage, _path);
        break;
      case sdf::GeometryType::PLANE:
      case sdf::GeometryType::ELLIPSOID:
      case sdf::GeometryType::HEIGHTMAP:
      default:
        std::cerr << "Geometry type is either invalid or not supported\n";
    }

    return typeParsed;
  }
}

