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

#include <iostream>
#include <string>

#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/tokens.h>

#include "sdf/World.hh"
#include "sdf_usd_parser/model.hh"

namespace usd
{
  bool ParseSdfWorld(const sdf::World &_world, pxr::UsdStageRefPtr &_stage,
      const std::string &_path)
  {
    auto usdWorldPrim = _stage->DefinePrim(pxr::SdfPath(_path));

    _stage->SetMetadata(pxr::UsdGeomTokens->upAxis, pxr::UsdGeomTokens->z);

    // parse all of the world's models and convert them to USD
    for (uint64_t i = 0; i < _world.ModelCount(); ++i)
    {
      const auto model = *(_world.ModelByIndex(i));
      const auto modelPath = std::string(_path + "/" + model.Name());
      if (!ParseSdfModel(model, _stage, modelPath))
      {
        std::cerr << "Error parsing model [" << model.Name() << "]\n";
        return false;
      }
    }

    // TODO(adlarkin) finish parsing other elements in _world

    return true;
  }
}
