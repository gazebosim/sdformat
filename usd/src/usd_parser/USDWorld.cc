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

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <gz/common/Filesystem.hh>
#include <gz/common/Util.hh>

#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usdGeom/camera.h>
#include <pxr/usd/usdGeom/gprim.h>
#include <pxr/usd/usdGeom/scope.h>
#include <pxr/usd/usdLux/boundableLightBase.h>
#include <pxr/usd/usdLux/nonboundableLightBase.h>
#include <pxr/usd/usdPhysics/rigidBodyAPI.h>
#include <pxr/usd/usdPhysics/scene.h>
#include <pxr/usd/usdPhysics/joint.h>
#include <pxr/usd/usdShade/material.h>
#pragma pop_macro ("__DEPRECATED")

#include "sdf/usd/usd_parser/USDData.hh"
#include "sdf/usd/usd_parser/USDStage.hh"
#include "sdf/usd/usd_parser/USDTransforms.hh"

#include "USDJoints.hh"
#include "USDLights.hh"
#include "USDPhysics.hh"
#include "USDSensors.hh"
#include "USDLinks.hh"

#include "sdf/Collision.hh"
#include "sdf/Light.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"
#include "sdf/Plugin.hh"
#include "sdf/Visual.hh"
#include "sdf/World.hh"

namespace sdf
{
inline namespace SDF_VERSION_NAMESPACE {
namespace usd
{
  UsdErrors parseUSDWorld(const std::string &_inputFileName,
      bool _useGazeboPlugins, sdf::World &_world)
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
    std::string currentModelName;

    // USD link may have scale, store this value to apply this to the sdf visual
    // the key is the name of the link and the value is the scale value
    std::map<std::string, gz::math::Vector3d> linkScaleMap;

    auto range = pxr::UsdPrimRange::Stage(reference);
    for (const auto &prim : range)
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
        gz::common::split(primPath, "/");

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
        currentModelName = primPathTokens[0];

        gz::math::Pose3d pose;
        gz::math::Vector3d scale{1, 1, 1};

        GetTransform(
          prim,
          usdData,
          pose,
          scale,
          model.Name());

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
      // https://github.com/gazebosim/sdformat/issues/927
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

      sdf::Model *modelPtr = nullptr;
      if (!currentModelName.empty())
      {
        modelPtr = _world.ModelByName(currentModelName);
        if (!modelPtr)
        {
          errors.push_back(UsdError(UsdErrorCode::USD_TO_SDF_PARSING_ERROR,
                "Unable to find a sdf::Model named [" + currentModelName +
                "] in world named [" + _world.Name() +
                "], but a sdf::Model with this name should exist."));
          return errors;
        }
      }

      if (_useGazeboPlugins && primType == "RosDifferentialBase")
      {
        if (!modelPtr)
        {
          errors.push_back(UsdError(UsdErrorCode::USD_TO_SDF_PARSING_ERROR,
            "Unable to store RosDifferentialBase in a DiffDrive plugin "
            "because the corresponding sdf::Model object wasn't found."));
          return errors;
        }

        auto leftWheelAttr = prim.GetAttribute(
          pxr::TfToken("leftWheelJointName"));
        auto rightWheelAttr = prim.GetAttribute(
          pxr::TfToken("rightWheelJointName"));
        auto wheelBaseAttr = prim.GetAttribute(
          pxr::TfToken("wheelBase"));
        auto wheelRadiusAttr = prim.GetAttribute(
          pxr::TfToken("wheelRadius"));

        sdf::Plugin diffDrivePlugin;
        diffDrivePlugin.SetName("gz::sim::systems::DiffDrive");
        diffDrivePlugin.SetFilename("gz-sim-diff-drive-system");

        std::string leftWheelName;
        std::string rightWheelName;
        float wheelBase;
        float wheelRadius;
        wheelBaseAttr.Get<float>(&wheelBase);
        wheelRadiusAttr.Get<float>(&wheelRadius);
        leftWheelAttr.Get<std::string>(&leftWheelName);
        rightWheelAttr.Get<std::string>(&rightWheelName);

        sdf::ElementPtr leftJointContent(new sdf::Element);
        leftJointContent->SetName("left_joint");
        leftJointContent->AddValue("string", "", false);
        leftJointContent->Set(leftWheelName + "_joint");
        diffDrivePlugin.InsertContent(leftJointContent);

        sdf::ElementPtr rightJointContent(new sdf::Element);
        rightJointContent->SetName("right_joint");
        rightJointContent->AddValue("string", "", false);
        rightJointContent->Set(rightWheelName + "_joint");
        diffDrivePlugin.InsertContent(rightJointContent);

        sdf::ElementPtr wheelSeparationContent(new sdf::Element);
        wheelSeparationContent->SetName("wheel_separation");
        wheelSeparationContent->AddValue("float", "0.0", false);
        wheelSeparationContent->Set(wheelBase);
        diffDrivePlugin.InsertContent(wheelSeparationContent);

        sdf::ElementPtr wheelRadiusContent(new sdf::Element);
        wheelRadiusContent->SetName("wheel_radius");
        wheelRadiusContent->AddValue("float", "0.0", false);
        wheelRadiusContent->Set(wheelRadius);
        diffDrivePlugin.InsertContent(wheelRadiusContent);

        modelPtr->AddPlugin(diffDrivePlugin);
        continue;
      }

      if (prim.IsA<pxr::UsdLuxBoundableLightBase>() ||
          prim.IsA<pxr::UsdLuxNonboundableLightBase>())
      {
        auto light = ParseUSDLights(prim, usdData, linkName);
        if (light)
        {
          light->SetName(primName);

          // assume this light belongs to the world unless the corresponding
          // model/link for this light are found
          bool worldLight = true;

          // if the light prim we are parsing has no parent (or if its parent
          // is the root prim), this means the light belongs to the world
          const bool noModelAncestor = !prim.GetParent() ||
            (prim.GetParent().GetName().GetString() == "/");
          if (!noModelAncestor && modelPtr)
          {
            if (auto link =
                modelPtr->LinkByName(gz::common::basename(linkName)))
            {
              link->AddLight(light.value());
              worldLight = false;
            }
          }

          if (worldLight)
          {
            _world.AddLight(light.value());
          }
        }
        continue;
      }

      if (prim.IsA<pxr::UsdPhysicsJoint>())
      {
        if (!modelPtr)
        {
          errors.push_back(UsdError(UsdErrorCode::USD_TO_SDF_PARSING_ERROR,
            "Unable to parse joint corresponding to USD prim [" +
            std::string(prim.GetName()) +
            "] because the corresponding sdf::Model object wasn't found."));
          return errors;
        }
        sdf::Joint joint;
        auto errorsJoint = ParseJoints(prim, usdData, joint);
        if (!errorsJoint.empty())
        {
          errors.push_back(UsdError(UsdErrorCode::USD_TO_SDF_PARSING_ERROR,
            "Unable to find parse UsdPhysicsJoint [" +
            std::string(prim.GetName()) + "]"));
          return errors;
        }
        modelPtr->AddJoint(joint);
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

      if (prim.IsA<pxr::UsdGeomCamera>() || primType == "Lidar")
      {
        if (!linkName.empty())
        {
          auto sensor = ParseSensors(prim, usdData);
          auto link = modelPtr->LinkByName(linkName);
          if (link != nullptr)
          {
            link->AddSensor(sensor);
          }
        }
        continue;
      }

      if (!prim.IsA<pxr::UsdGeomGprim>() && !(primType == "Plane"))
      {
        continue;
      }

      if (!modelPtr)
      {
        errors.push_back(UsdError(UsdErrorCode::USD_TO_SDF_PARSING_ERROR,
              "Unable to parse link named [" + linkName +
              "] because the corresponding sdf::Model object wasn't found."));
        return errors;
      }

      std::optional<sdf::Link> optionalLink;
      if (auto linkInserted =
          modelPtr->LinkByName(gz::common::basename(linkName)))
      {
        optionalLink = *linkInserted;
        auto scale = linkScaleMap.find(linkName);
        if (scale == linkScaleMap.end())
        {
          scale = linkScaleMap.insert(
              {linkName, gz::math::Vector3d(1, 1, 1)}).first;
        }
        sdf::usd::ParseUSDLinks(
          prim, linkName, optionalLink, usdData, scale->second);
        *linkInserted = optionalLink.value();
        linkScaleMap[linkName] = scale->second;
      }
      else
      {
        gz::math::Vector3d scale{1, 1, 1};

        sdf::usd::ParseUSDLinks(prim, linkName, optionalLink, usdData, scale);
        linkScaleMap[linkName] = scale;

        if (optionalLink && !optionalLink->Name().empty() &&
            !modelPtr->LinkByName(optionalLink->Name()))
        {
          modelPtr->AddLink(optionalLink.value());
        }
      }
    }

    for (unsigned int i = 0; i < _world.LightCount(); ++i)
    {
      auto light = _world.LightByIndex(i);
      light->SetName(gz::common::basename(light->Name()));
    }

    for (unsigned int i = 0; i < _world.ModelCount(); ++i)
    {
      const auto m = _world.ModelByIndex(i);

      // We might include some empty models
      // for example: some models create a world path to set up physics
      // but there is no model data inside
      // TODO(ahcorde) Add a RemoveModelByName() method in sdf::World
      if (m->LinkCount() == 0)
      {
        sdf::Link emptyLink;
        emptyLink.SetName("emptyLink");
        m->AddLink(emptyLink);
      }
    }

    if (_useGazeboPlugins)
    {
      // Add some plugins to run the Gazebo Sim simulation
      sdf::Plugin physicsPlugin;
      physicsPlugin.SetName("gz::sim::systems::Physics");
      physicsPlugin.SetFilename("gz-sim-physics-system");
      _world.AddPlugin(physicsPlugin);

      sdf::Plugin sensorsPlugin;
      sensorsPlugin.SetName("gz::sim::systems::Sensors");
      sensorsPlugin.SetFilename("gz-sim-sensors-system");
      _world.AddPlugin(sensorsPlugin);

      sdf::Plugin userCommandsPlugin;
      userCommandsPlugin.SetName("gz::sim::systems::UserCommands");
      userCommandsPlugin.SetFilename("gz-sim-user-commands-system");
      _world.AddPlugin(userCommandsPlugin);

      sdf::Plugin sceneBroadcasterPlugin;
      sceneBroadcasterPlugin.SetName(
        "gz::sim::systems::SceneBroadcaster");
      sceneBroadcasterPlugin.SetFilename(
        "gz-sim-scene-broadcaster-system");
      _world.AddPlugin(sceneBroadcasterPlugin);
    }

    return errors;
  }
}
}
}
