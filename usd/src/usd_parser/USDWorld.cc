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
#include "USDWorld.hh"

#include <string>

#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usdPhysics/scene.h>
#pragma pop_macro ("__DEPRECATED")

#include "sdf/usd/usd_parser/USDData.hh"
#include "sdf/usd/usd_parser/USDStage.hh"
#include "USDPhysics.hh"

#include "sdf/World.hh"

namespace sdf
{
inline namespace SDF_VERSION_NAMESPACE {
namespace usd
{
  UsdErrors parseUSDWorld(const std::string &_inputFileName,
    sdf::World &_world)
  {
    UsdErrors errors;
    USDData usdData(_inputFileName);
    usdData.Init();
    usdData.ParseMaterials();

    auto reference = pxr::UsdStage::Open(_inputFileName);
    if (!reference)
    {
      errors.emplace_back(UsdError(
        UsdErrorCode::INVALID_USD_FILE,
        "Unable to open [" + _inputFileName + "]"));
      return errors;
    }
    std::string worldName = reference->GetDefaultPrim().GetName().GetText();
    if (!worldName.empty())
    {
      _world.SetName("world_name");
    }
    else
    {
      _world.SetName(worldName + "_world");
    }

    auto range = pxr::UsdPrimRange::Stage(reference);
    for (auto const &prim : range)
    {
      std::string primName = prim.GetName();

      if (prim.IsA<pxr::UsdPhysicsScene>())
      {
        std::pair<std::string, std::shared_ptr<USDStage>> data =
          usdData.FindStage(primName);
        if (!data.second)
        {
          errors.push_back(UsdError(UsdErrorCode::INVALID_PRIM_PATH,
                "Unable to retrieve the pxr::UsdPhysicsScene named ["
                + primName + "]"));
          return errors;
        }

        ParseUSDPhysicsScene(pxr::UsdPhysicsScene(prim), _world,
            data.second->MetersPerUnit());
        continue;
      }
    }
    return errors;
  }
}
}
}
