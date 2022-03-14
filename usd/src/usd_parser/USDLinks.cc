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

#include "USDLinks.hh"

#include <string>

#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/usd/usdGeom/cube.h>
#include <pxr/usd/usdGeom/cylinder.h>
#include <pxr/usd/usdGeom/gprim.h>
#include <pxr/usd/usdGeom/mesh.h>
#include <pxr/usd/usdGeom/sphere.h>
#pragma pop_macro ("__DEPRECATED")

#include <ignition/common/Filesystem.hh>

#include "sdf/Geometry.hh"
#include "sdf/Link.hh"

namespace sdf
{
// Inline bracke to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
//
namespace usd
{
std::string ParseUSDLinks(
  const pxr::UsdPrim &_prim,
  const std::string &_nameLink,
  sdf::Link &_link,
  USDData &_usdData,
  int &_skipPrim)
{
  std::string primNameStr = _prim.GetPath().GetName();
  std::string primPathStr = pxr::TfStringify(_prim.GetPath());
  std::string primType = _prim.GetPrimTypeInfo().GetTypeName().GetText();

  std::pair<std::string, std::shared_ptr<USDStage>> data =
    _usdData.FindStage(primNameStr);

  double metersPerUnit = data.second->MetersPerUnit();

  if (_link.Name().empty())
  {
    _link.SetName(ignition::common::basename(_nameLink));
  }

  sdf::Geometry geom;
  if (_prim.IsA<pxr::UsdGeomSphere>() ||
      _prim.IsA<pxr::UsdGeomCylinder>() ||
      _prim.IsA<pxr::UsdGeomCube>() ||
      _prim.IsA<pxr::UsdGeomMesh>() ||
      primType == "Plane")
  {
    // std::shared_ptr<sdf::Visual> vis;
    // vis = std::make_shared<sdf::Visual>();
  }

  return _link.Name();
}
}
}
}
