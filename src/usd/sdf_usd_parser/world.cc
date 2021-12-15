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

#include "sdf_usd_parser/world.hh"

#include <algorithm>
#include <iostream>
#include <string>

#include <pxr/base/gf/vec3f.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/tokens.h>
#include <pxr/usd/usdPhysics/scene.h>

#include "sdf/World.hh"
#include "sdf_usd_parser/light.hh"
#include "sdf_usd_parser/model.hh"

namespace usd
{
  // TODO(ahcorde): Move this function to common::Util.hh
  void removeSpaces(std::string &_str)
  {
    _str.erase(
      std::remove_if(
        _str.begin(),
        _str.end(),
        [](unsigned char x)
        {
          return std::isspace(x);
        }),
      _str.end());
  }

  bool ParseSdfWorld(const sdf::World &_world, pxr::UsdStageRefPtr &_stage,
      const std::string &_path)
  {
    _stage->SetMetadata(pxr::UsdGeomTokens->upAxis, pxr::UsdGeomTokens->z);

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
      removeSpaces(modelPath);
      if (!ParseSdfModel(model, _stage, modelPath, worldPrimPath))
      {
        std::cerr << "Error parsing model [" << model.Name() << "]\n";
        return false;
      }
    }

    for (uint64_t i = 0; i < _world.LightCount(); ++i)
    {
      const auto light = *(_world.LightByIndex(i));
      auto lightPath = std::string(_path + "/" + light.Name());
      removeSpaces(lightPath);
      if (!ParseSdfLight(light, _stage, lightPath))
      {
        std::cerr << "Error parsing light [" << light.Name() << "]\n";
        return false;
      }
    }

    // TODO(adlarkin) finish parsing other elements in _world

    return true;
  }
}
