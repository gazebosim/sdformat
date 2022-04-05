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
#include <pxr/usd/usdGeom/scope.h>
#include <pxr/usd/usdLux/boundableLightBase.h>
#include <pxr/usd/usdLux/nonboundableLightBase.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usdPhysics/rigidBodyAPI.h>
#include <pxr/usd/usdPhysics/scene.h>
#include <pxr/usd/usdShade/material.h>
#pragma pop_macro ("__DEPRECATED")

#include "sdf/usd/usd_parser/USDData.hh"
#include "sdf/usd/usd_parser/USDStage.hh"
#include "sdf/usd/usd_parser/USDTransforms.hh"

#include "USDLights.hh"
#include "USDPhysics.hh"
#include "USDLinks.hh"

#include "sdf/Light.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"
#include "sdf/Plugin.hh"
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
    errors = usdData.Init();
    if (!errors.empty())
      return errors;
    errors = usdData.ParseMaterials();
    if (!errors.empty())
      return errors;

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

    int skipPrims = 0;

    auto range = pxr::UsdPrimRange::Stage(reference);
    for (auto const &prim : range)
    {
      if (skipPrims)
      {
        --skipPrims;
        continue;
      }

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

      // This assumption on the scene graph wouldn't hold if the usd does
      // not come from Isaac Sim
      if (primPathTokens.size() == 1 && !prim.IsA<pxr::UsdGeomCamera>()
          && !prim.IsA<pxr::UsdGeomScope>()
          && !prim.IsA<pxr::UsdPhysicsScene>()
          && !prim.IsA<pxr::UsdLuxBoundableLightBase>()
          && !prim.IsA<pxr::UsdLuxNonboundableLightBase>())
      {
        sdf::Model model = sdf::Model();
        model.SetName(primPathTokens[0]);

        ignition::math::Pose3d pose;
        ignition::math::Vector3d scale{1, 1, 1};

        GetTransform(
          prim,
          usdData,
          pose,
          scale,
          model.Name());

          std::cerr << primPathTokens[0] << " - " << !prim.HasAPI<pxr::UsdPhysicsRigidBodyAPI>() << '\n';

        model.SetRawPose(pose);
        model.SetStatic(!prim.HasAPI<pxr::UsdPhysicsRigidBodyAPI>());

        _world.AddModel(model);
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
      // TODO(adlarkin) find a better way to get root model prims/parent prims
      // of lights attached to the stage: see
      // https://github.com/ignitionrobotics/sdformat/issues/927
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
        if (!shortName)
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
          _world.AddLight(light.value());
          // TODO(ahcorde) Include lights which are inside links
        }
        continue;
      }
      // TODO(anyone) support converting other USD light types

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

      if (!prim.IsA<pxr::UsdGeomGprim>() && !(primType == "Plane"))
      {
        continue;
      }

      auto linkInserted = modelPtr->LinkByName(linkName);
      if (linkInserted)
      {
        sdf::usd::ParseUSDLinks(prim, linkName, linkInserted, usdData, skipPrims);
      }
      else
      {
        sdf::Link * link = nullptr;
        link = sdf::usd::ParseUSDLinks(prim, linkName, link, usdData, skipPrims);

        if (link != nullptr && !link->Name().empty())
        {
          const auto l = modelPtr->LinkByName(link->Name());
          if (l == nullptr)
          {
            std::cerr << "Added "<< link->Name() << '\n';
            modelPtr->AddLink(*link);
          }
        }
        if (link == nullptr)
          delete link;
      }
    }
    for (unsigned int i = 0; i < _world.LightCount(); ++i)
    {
      auto light = _world.LightByIndex(i);
      light->SetName(ignition::common::basename(light->Name()));
    }
    for (unsigned int i = 0; i < _world.ModelCount(); ++i)
    {
      const auto m = _world.ModelByIndex(i);

      // We might include some empty models
      // for example: same models create a world path to set up physics
      // but there is no model data inside
      // TODO(ahcorde) Add a RemoveModelByName() method in sdf::World
      if (m->LinkCount() == 0)
      {
        sdf::Link emptyLink;
        emptyLink.SetName("emptyLink");
        m->AddLink(emptyLink);
      }

      m->SetName(ignition::common::basename(m->Name()));
      for (unsigned int j = 0; j < m->LinkCount(); ++j)
      {
        auto link = m->LinkByIndex(j);
        link->SetName(ignition::common::basename(link->Name()));
      }
    }

    // TODO(ahcorde): Remove this loop here, I added this here to avoid
    // errors, a model should have link. This will be added in a follow up PR.
    for (unsigned int i = 0; i < _world.ModelCount(); ++i)
    {
      auto m = _world.ModelByIndex(i);
      if (m->LinkCount() == 0)
      {
        sdf::Link link;
        link.SetName("empty_link");
        m->AddLink(link);
      }
    }

    // Add some plugins to run the Ignition Gazebo simulation
    sdf::Plugin physicsPlugin;
    physicsPlugin.SetName("ignition::gazebo::systems::Physics");
    physicsPlugin.SetFilename("ignition-gazebo-physics-system");
    _world.AddPlugin(physicsPlugin);

    sdf::Plugin sensorsPlugin;
    sensorsPlugin.SetName("ignition::gazebo::systems::Sensors");
    sensorsPlugin.SetFilename("ignition-gazebo-sensors-system");
    _world.AddPlugin(sensorsPlugin);

    sdf::Plugin userCommandsPlugin;
    userCommandsPlugin.SetName("ignition::gazebo::systems::UserCommands");
    userCommandsPlugin.SetFilename("ignition-gazebo-user-commands-system");
    _world.AddPlugin(userCommandsPlugin);

    sdf::Plugin sceneBroadcasterPlugin;
    sceneBroadcasterPlugin.SetName(
      "ignition::gazebo::systems::SceneBroadcaster");
    sceneBroadcasterPlugin.SetFilename(
      "ignition-gazebo-scene-broadcaster-system");
    _world.AddPlugin(sceneBroadcasterPlugin);

    return errors;
  }
}
}
}
