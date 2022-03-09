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

#include "Lights.hh"
#include "Physics.hh"

#include <ignition/common/Util.hh>

#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/usd/usdGeom/gprim.h>
#include <pxr/usd/usdLux/boundableLightBase.h>
#include <pxr/usd/usdLux/nonboundableLightBase.h>
#include <pxr/usd/usdPhysics/scene.h>
#include <pxr/usd/usdShade/material.h>
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

    _world->_worldName = referencee->GetDefaultPrim().GetName().GetText();

    std::string linkName;

    for (auto const &prim : range)
    {
      // Skip materials, the data is already available in the USDData class
      if (prim.IsA<pxr::UsdShadeMaterial>() || prim.IsA<pxr::UsdShadeShader>())
      {
        continue;
      }

      std::string primName = prim.GetPath().GetName();
      std::string primPath = pxr::TfStringify(prim.GetPath());
      std::string primType = pxr::TfStringify(prim.GetPath());

      std::vector<std::string> primPathTokens =
        ignition::common::split(primPath, "/");

      // In general USD models used in Issac Sim define the model path
      // under a root path for example:
      //  -> /robot_name/robot_name_link0
      // But sometimes for enviroments it uses just a simple path:
      //  -> /ground_plan
      //  -> /wall_0
      // the shortName variable defines if this is the first case when it's
      // False or when it's true then it's the second case.
      if (primPathTokens.size() >= 2)
      {
        bool shortName = false;
        if (primPathTokens.size() == 2)
        {
          if (prim.IsA<pxr::UsdGeomGprim>() || (primType == "Plane"))
          {
            if (primName != "imu")
            {
              linkName = "/" + primPathTokens[0];
              shortName = true;
            }
          }
        }
        if(!shortName)
        {
          linkName = "/" + primPathTokens[0] + "/" + primPathTokens[1];
        }
      }

      if (prim.IsA<pxr::UsdLuxBoundableLightBase>() ||
          prim.IsA<pxr::UsdLuxNonboundableLightBase>())
      {
        auto light = ParseLights(prim, linkName);
        if (light)
        {
          _world->lights.insert(
            std::pair<std::string, std::shared_ptr<sdf::Light>>
              (primName, light));
          // TODO(ahcorde): Include lights which are inside links
        }
        continue;
      }

      if (prim.IsA<pxr::UsdPhysicsScene>())
      {
        std::pair<std::string, std::shared_ptr<USDStage>> data =
          usdData.FindStage(prim.GetPath().GetName());

        ParsePhysicsScene(prim, _world, data.second->MetersPerUnit());
        continue;
      }
    }

    for (auto & light : _world->lights)
    {
      std::cout << "-------------Lights--------------" << std::endl;
      std::cout << light.second->Name() << std::endl;
    }

    return errors;
  }
}
}
}
