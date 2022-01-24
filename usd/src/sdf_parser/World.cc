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

#include "sdf/usd/sdf_parser/World.hh"

#include <iostream>
#include <string>

#include <pxr/base/gf/vec3f.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/prim.h>
#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/usd/usd/stage.h>
#pragma pop_macro ("__DEPRECATED")
#include <pxr/usd/usdGeom/tokens.h>
#include <pxr/usd/usdPhysics/scene.h>

#include "sdf/World.hh"

namespace sdf
{
// Inline bracke to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
//
namespace usd
{
  sdf::Errors ParseSdfWorld(const sdf::World &_world, pxr::UsdStageRefPtr &_stage,
      const std::string &_path)
  {
    sdf::Errors errors;
    _stage->SetMetadata(pxr::UsdGeomTokens->upAxis, pxr::UsdGeomTokens->z);
    _stage->SetEndTimeCode(100);
    _stage->SetMetadata(pxr::TfToken("metersPerUnit"), 1.0);
    _stage->SetStartTimeCode(0);
    _stage->SetTimeCodesPerSecond(24);

    const pxr::SdfPath worldPrimPath(_path);
    auto usdWorldPrim = _stage->DefinePrim(worldPrimPath);

    auto usdPhysics = pxr::UsdPhysicsScene::Define(_stage,
        pxr::SdfPath(_path + "/physics"));
    const auto &sdfWorldGravity = _world.Gravity();
    const auto normalizedGravity = sdfWorldGravity.Normalized();
    usdPhysics.CreateGravityDirectionAttr().Set(pxr::GfVec3f(
          normalizedGravity.X(), normalizedGravity.Y(), normalizedGravity.Z()));
    usdPhysics.CreateGravityMagnitudeAttr().Set(
        static_cast<float>(sdfWorldGravity.Length()));

    // TODO(ahcorde) Add parser
    std::cerr << "Parser for a sdf world only parses physics information at "
              << "the moment. Models and lights that are children of the world "
              << "are currently being ignored.\n";

    return errors;
  }
}
}
}
