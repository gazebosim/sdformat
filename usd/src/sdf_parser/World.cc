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

#include <ignition/common/Util.hh>

// TODO(ahcorde):this is to remove deprecated "warnings" in usd, these warnings
// are reported using #pragma message so normal diagnostic flags cannot remove
// them. This workaround requires this block to be used whenever usd is
// included.
#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/base/gf/vec3f.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/tokens.h>
#include <pxr/usd/usdPhysics/scene.h>
#pragma pop_macro ("__DEPRECATED")

#include "sdf/World.hh"
#include "sdf/usd/sdf_parser/Light.hh"
#include "sdf/usd/sdf_parser/Model.hh"

namespace sdf
{
// Inline bracke to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
//
namespace usd
{
  UsdErrors ParseSdfWorld(const sdf::World &_world,
    pxr::UsdStageRefPtr &_stage, const std::string &_path)
  {
    UsdErrors errors;
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

    // parse all of the world's models and convert them to USD
    for (uint64_t i = 0; i < _world.ModelCount(); ++i)
    {
      const auto model = *(_world.ModelByIndex(i));
      auto modelPath = std::string(_path + "/" + model.Name());
      modelPath = ignition::common::replaceAll(modelPath, " ", "");
      sdf::Errors modelErrors =
        ParseSdfModel(model, _stage, modelPath, worldPrimPath);
      if (!modelErrors.empty())
      {
        errors.push_back(sdf::Error(sdf::ErrorCode::ATTRIBUTE_INCORRECT_TYPE,
              "Error parsing model [" + model.Name() + "]"));
        errors.insert(errors.end(), modelErrors.begin(), modelErrors.end());
      }
    }

    for (uint64_t i = 0; i < _world.LightCount(); ++i)
    {
      const auto light = *(_world.LightByIndex(i));
      auto lightPath = std::string(_path + "/" + light.Name());
      lightPath = ignition::common::replaceAll(lightPath, " ", "");
      sdf::Errors lightErrors = ParseSdfLight(light, _stage, lightPath);
      if (!lightErrors.empty())
      {
        errors.push_back(sdf::Error(sdf::ErrorCode::ATTRIBUTE_INCORRECT_TYPE,
              "Error parsing light [" + light.Name() + "]"));
        errors.insert(errors.end(), lightErrors.begin(), lightErrors.end());
      }
    }

    return errors;
  }
}
}
}
