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

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <ignition/common/Util.hh>

#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/usd/usdGeom/camera.h>
#include <pxr/usd/usdGeom/gprim.h>
#include <pxr/usd/usdLux/boundableLightBase.h>
#include <pxr/usd/usdLux/nonboundableLightBase.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usdPhysics/scene.h>
#include <pxr/usd/usdShade/material.h>
#pragma pop_macro ("__DEPRECATED")

#include "sdf/usd/usd_parser/USDData.hh"
#include "sdf/usd/usd_parser/USDStage.hh"
#include "sdf/usd/usd_parser/USDTransforms.hh"

#include "USDLights.hh"
#include "USDPhysics.hh"

#include "sdf/Model.hh"
#include "sdf/Light.hh"
#include "sdf/Link.hh"
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

    sdf::Model model;
    sdf::Model * modelPtr;

    auto reference = pxr::UsdStage::Open(_inputFileName);
    if (!reference)
    {
      errors.emplace_back(UsdErrorCode::INVALID_USD_FILE,
        "Unable to open [" + _inputFileName + "]");
      return errors;
    }
    std::string worldName = reference->GetDefaultPrim().GetName().GetText();
    if (worldName.empty())
    {
      _world.SetName("world_name");
    }
    else
    {
      _world.SetName(worldName + "_world");
    }

    std::string linkName;

    auto range = pxr::UsdPrimRange::Stage(reference);
    for (auto const &prim : range)
    {
      // Skip materials, the data is already available in the USDData class
      if (prim.IsA<pxr::UsdShadeMaterial>() || prim.IsA<pxr::UsdShadeShader>())
      {
        continue;
      }

      std::string primName = prim.GetName();
      std::string primPath = pxr::TfStringify(prim.GetPath());
      std::string primType = prim.GetPrimTypeInfo().GetTypeName().GetText();

      std::vector<std::string> primPathTokens =
        ignition::common::split(primPath, "/");

      // This assumption on the scene graph it wouldn't hold if the usd does
      // not come from Isaac Sim
      if (primPathTokens.size() == 1 && !prim.IsA<pxr::UsdGeomCamera>()
          && !prim.IsA<pxr::UsdPhysicsScene>()
          && !prim.IsA<pxr::UsdLuxBoundableLightBase>()
          && !prim.IsA<pxr::UsdLuxNonboundableLightBase>())
      {
        model = sdf::Model();
        model.SetName(primPathTokens[0]);
        _world.AddModel(model);
        modelPtr = _world.ModelByName(primPathTokens[0]);

        ignition::math::Pose3d pose;
        ignition::math::Vector3d scale{1, 1, 1};

        GetTransform(
          prim,
          usdData,
          pose,
          scale,
          model.Name());
        modelPtr->SetRawPose(pose);
      }

      // In general USD models used in Issac Sim define the model path
      // under a root path for example:
      //  -> /robot_name/robot_name_link0
      // But sometimes for enviroments it uses just a simple path:
      //  -> /ground_plane
      //  -> /wall_0
      // the shortName variable defines if this is the first case when it's
      // False or when it's true then it's the second case.
      // This conversion might only work with Issac Sim USDs
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
        auto light = ParseUSDLights(prim, usdData, linkName);
        light->SetName(primName);
        if (light)
        {
          _world.AddLight(*light.get());
          // TODO(ahcorde): Include lights which are inside links
        }
        continue;
      }

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

    for (unsigned int i = 0; i < _world.LightCount(); ++i)
    {
      std::cout << "-------------Lights--------------" << std::endl;
      std::cout << _world.LightByIndex(i)->Name() << std::endl;
    }

    std::cout << "-------------Models--------------" << std::endl;
    for (unsigned int i = 0; i < _world.ModelCount(); ++i)
    {
      auto m = _world.ModelByIndex(i);
      std::cout << m->Name() << std::endl;

      // TODO(ahcorde): Remove this link here, I added this here to avoid
      // errors. convert `m` in const.
      sdf::Link link;
      link.SetName("empty_link");
      m->AddLink(link);
    }

    return errors;
  }
}
}
}
