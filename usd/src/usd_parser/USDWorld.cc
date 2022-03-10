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

#include "sdf/usd/usd_parser/USDData.hh"
#include "sdf/usd/usd_parser/USDStage.hh"

#include "USDLights.hh"
#include "USDPhysics.hh"

#include <ignition/common/Util.hh>

#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/usd/usdGeom/camera.h>
#include <pxr/usd/usdGeom/gprim.h>
#include <pxr/usd/usdLux/boundableLightBase.h>
#include <pxr/usd/usdLux/nonboundableLightBase.h>
#include <pxr/usd/usdPhysics/scene.h>
#include <pxr/usd/usdShade/material.h>
#pragma pop_macro ("__DEPRECATED")

#include "sdf/usd/usd_parser/USDData.hh"
#include "sdf/usd/usd_parser/USDStage.hh"
#include "sdf/usd/usd_parser/USDTransforms.hh"
#include "USDPhysics.hh"
#include "USDLinks.hh"

#include "usd_model/LinkInterface.hh"
#include "usd_model/ModelInterface.hh"
#include "usd_model/WorldInterface.hh"

namespace sdf
{
inline namespace SDF_VERSION_NAMESPACE {
namespace usd
{
  UsdErrors parseUSDWorld(const std::string &_inputFileName,
    std::shared_ptr<WorldInterface> &_world)
  {
    UsdErrors errors;
    USDData usdData(_inputFileName);
    usdData.Init();
    usdData.ParseMaterials();

    std::shared_ptr<ModelInterface> model;

    auto reference = pxr::UsdStage::Open(_inputFileName);
    if (!reference)
    {
      errors.emplace_back(UsdError(
        UsdErrorCode::INVALID_USD_FILE,
        "Unable to open [" + _inputFileName + "]"));
      return errors;
    }
    _world->worldName = reference->GetDefaultPrim().GetName().GetText();

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
      std::string primType = pxr::TfStringify(prim.GetPath());

      std::vector<std::string> primPathTokens =
        ignition::common::split(primPath, "/");

      if (primPathTokens.size() == 1 && !prim.IsA<pxr::UsdGeomCamera>())
      {
        model = std::make_shared<ModelInterface>();
        model->Clear();
        model->name = primPathTokens[0];
        _world->models.push_back(model);

        ignition::math::Pose3d pose;
        ignition::math::Vector3d scale{1, 1, 1};

        GetTransform(
          prim,
          usdData,
          pose,
          scale,
          model->name);
        model->pose = pose;
      }

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
        auto light = ParseUSDLights(prim, usdData, linkName);
        if (light)
        {
          _world->lights.insert(
            std::pair<std::string, std::shared_ptr<sdf::Light>>
              (primName, light));
          _world->models.pop_back();
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

        ParseUSDPhysicsScene(prim, _world, data.second->MetersPerUnit());
        _world->models.pop_back();
        continue;
      }

      if (!prim.IsA<pxr::UsdGeomGprim>() && !(primType == "Plane"))
      {
        continue;
      }

      std::shared_ptr<LinkInterface> link = nullptr;
      auto it = model->links.find(linkName);
      if (it != model->links.end())
      {
        sdf::usd::ParseUSDLinks(prim, linkName, it->second, usdData, skipPrims);
      }
      else
      {
        sdf::usd::ParseUSDLinks(prim, linkName, link, usdData, skipPrims);

        if (link)
        {
          model->links.insert(make_pair(linkName, link));
        }
      }
    }

    ////////////////////////////////////////////////////////////////////////////

    std::cout << "-------------Lights--------------" << std::endl;
    for (auto & light : _world->lights)
    {
      std::cout << light.second->Name() << std::endl;
    }
    std::cout << "---------------------------" << std::endl;

    std::cout << "-------------Models--------------" << std::endl;
    for (auto & m : _world->models)
    {
      std::cout << m->Name() << std::endl;
      for (auto & link : m->links)
      {
        std::cout << "\t" << link.second->name << std::endl;
      }
    }
    std::cout << "---------------------------" << std::endl;

    ////////////////////////////////////////////////////////////////////////////

    for (auto & m : _world->models)
    {
      std::cout << m->Name() << std::endl;
      for (auto & link : m->links)
      {
        std::shared_ptr<sdf::Joint> joint = nullptr;
        joint = std::make_shared<sdf::Joint>();
        std::string joint_name = link.second->name;
        joint->SetName(joint_name);

        joint->SetType(sdf::JointType::FIXED);
        joint->SetParentLinkName("world");
        joint->SetChildLinkName(link.second->name);
        m->joints.insert(make_pair(joint->Name(), joint));
      }

      for (auto & joint : m->joints)
      {
        if (joint.second->ParentLinkName() == "world")
        {
          // insert <link name="world"/>
          std::shared_ptr<LinkInterface> worldLink =
            std::make_shared<LinkInterface>();
          worldLink->Clear();
          worldLink->name = "world";
          m->links.insert(make_pair(worldLink->name, worldLink));
        }
      }
    }

    for(auto iteratorModel = _world->models.begin();
        iteratorModel != _world->models.end();)
    {
      auto & m = *iteratorModel;

      if (m->links.size() == 0)
      {
         _world->models.erase(iteratorModel);
         continue;
      }

      // every link has children links and joints, but no parents, so we create a
      // local convenience data structure for keeping child->parent relations
      std::map<std::string, std::string> parent_link_tree;
      parent_link_tree.clear();

      UsdErrors initTreeError = m->InitTree(parent_link_tree);
      if (!initTreeError.empty())
      {
        errors.insert(errors.end(), initTreeError.begin(), initTreeError.end());
        errors.emplace_back(UsdError(UsdErrorCode::INVALID_PRIM_PATH,
              "Error initializing the tree [" + m->Name() + "]"));
        m.reset();
      }
      else
      {
        UsdErrors initRootError = m->InitRoot(parent_link_tree);
        if (!initRootError.empty())
        {
          errors.insert(errors.end(), initRootError.begin(), initRootError.end());
          errors.emplace_back(UsdError(UsdErrorCode::INVALID_PRIM_PATH,
                "Error initializing the root [" + m->Name() + "]"));
          m.reset();
        }
      }

      ++iteratorModel;
    }

    if (!errors.empty())
    {
      return errors;
    }

    return errors;
  }
}
}
}
