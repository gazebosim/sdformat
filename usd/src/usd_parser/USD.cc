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
#include "USD.hh"

#include "sdf/usd/usd_parser/USDData.hh"
#include "sdf/usd/usd_parser/USDStage.hh"
#include "Physics.hh"

#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/usd/usdPhysics/scene.h>
#pragma pop_macro ("__DEPRECATED")

#include <string>

namespace sdf
{
inline namespace SDF_VERSION_NAMESPACE {
namespace usd
{
  UsdErrors parseUSDWorld(
    const std::string &_inputFilename,
    std::shared_ptr<WorldInterface> &_world)
  {
    UsdErrors errors;
    USDData usdData(_inputFilename);
    usdData.Init();
    usdData.ParseMaterials();

    auto referencee = pxr::UsdStage::Open(_inputFilename);
    if (!referencee)
    {
      errors.emplace_back(UsdError(
        UsdErrorCode::INVALID_USD_FILE,
        "Unable to open [" + _inputFilename + "]"));
      return errors;
    }
    auto range = pxr::UsdPrimRange::Stage(referencee);

    _world->worldName = referencee->GetDefaultPrim().GetName().GetText();

    for (auto const &prim : range)
    {
      if (prim.IsA<pxr::UsdPhysicsScene>())
      {
        std::pair<std::string, std::shared_ptr<USDStage>> data =
          usdData.FindStage(prim.GetPath().GetName());

        ParsePhysicsScene(prim, _world, data.second->MetersPerUnit());
        continue;
      }
    }
    return errors;
  }
}
}
}
