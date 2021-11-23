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

#include <pxr/usd/sdf/reference.h>
#include <pxr/usd/usd/attribute.h>
#include <pxr/base/tf/stringUtils.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usd/primRange.h>
#include "pxr/usd/usdGeom/gprim.h"
#include "pxr/usd/usdPhysics/rigidBodyAPI.h"

// #include "pxr/usd/usdLux/light.h"

#include <pxr/usd/usdShade/material.h>
#include <pxr/usd/usdShade/materialBindingAPI.h>

#include "pxr/base/gf/vec3d.h"
#include "pxr/base/gf/matrix4d.h"
#include "pxr/base/gf/rotation.h"

#include "pxr/usd/usdPhysics/collisionGroup.h"
#include "pxr/usd/usdPhysics/fixedJoint.h"
#include "pxr/usd/usdPhysics/joint.h"
#include "pxr/usd/usdPhysics/scene.h"

#include "pxr/usd/usdShade/material.h"

#include "pxr/usd/usdLux/lightAPI.h"
#include "pxr/usd/usdLux/boundableLightBase.h"
#include "pxr/usd/usdLux/sphereLight.h"

#include "pxr/usd/usdGeom/camera.h"

// #include "pxr/base/tf/staticTokens.h"
// #include "pxr/usd/usdRi/tokens.h"

#include "usd_parser/parser_usd.hh"
#include "physics.hh"
#include "joints.hh"
#include "lights.hh"
#include "links.hh"
#include "sensors.hh"
#include "utils.hh"
#include "sdf/Console.hh"
#include "sdf/Mesh.hh"
#include "sdf/Pbr.hh"

#include <fstream>

#include <ignition/common/Util.hh>
#include <ignition/common/Mesh.hh>
#include <ignition/common/SubMesh.hh>
#include <ignition/common/ColladaLoader.hh>
#include <ignition/common/ColladaExporter.hh>
#include <ignition/common/Material.hh>

namespace usd {

std::vector<ModelInterfaceSharedPtr> parseUSD(const std::string &xml_string)
{
  std::vector<ModelInterfaceSharedPtr> models;
  ModelInterfaceSharedPtr model = std::make_shared<ModelInterface>();
  model->clear();
  models.push_back(model);

  auto referencee = pxr::UsdStage::Open(xml_string);
  if (!referencee)
  {
    return models;
  }

  auto range = pxr::UsdPrimRange::Stage(referencee);

  double metersPerUnit;
  std::string defaultName;
  referencee->GetMetadata<double>(pxr::TfToken("metersPerUnit"), &metersPerUnit);
  defaultName = referencee->GetDefaultPrim().GetName().GetText();
  pxr::TfToken axis;
  referencee->GetMetadata(pxr::UsdGeomTokens->upAxis, &axis);
  std::string upAxis = axis.GetText();
  std::cerr << "metersPerUnit" << metersPerUnit << '\n';
  std::cerr << "upAxis: " << upAxis << '\n';
  std::cerr << "defaultPrim" << defaultName << '\n';

  std::string modelName;
  std::string baseLink;
  std::string nameLink;

  model->world_name_ = defaultName;

  int skipNext = 0;

  std::map<std::string, std::shared_ptr<ignition::common::Material>> materials;
  std::map<std::string, sdf::Material> materialsSDF;

  for (auto const &prim : range)
  {
    std::string primName = pxr::TfStringify(prim.GetPath());
    if (prim.IsA<pxr::UsdShadeMaterial>())
    {
      std::string materialName = std::string(prim.GetName());

      if (materials.find(materialName) != materials.end())
      {
        continue;
      }
      std::cerr << "new material!!! " << materialName << '\n';
      sdf::Material material = ParseMaterial(prim);

      std::shared_ptr<ignition::common::Material> materialCommon
         = std::make_shared<ignition::common::Material>();

      materialCommon->SetEmissive(material.Emissive());
      materialCommon->SetDiffuse(material.Diffuse());
      const sdf::Pbr * pbr = material.PbrMaterial();
      if(pbr != nullptr)
      {
        const sdf::PbrWorkflow * pbrWorkflow = pbr->Workflow(sdf::PbrWorkflowType::METAL);
        if (pbrWorkflow != nullptr)
        {
          ignition::common::Pbr pbrCommon;
          pbrCommon.SetRoughness(pbrWorkflow->Roughness());
          pbrCommon.SetMetalness(pbrWorkflow->Metalness());
          if (!pbrWorkflow->MetalnessMap().empty())
          {
            pbrCommon.SetMetalnessMap(pbrWorkflow->MetalnessMap());
          }
          if (!pbrWorkflow->AlbedoMap().empty())
          {
            pbrCommon.SetAlbedoMap(pbrWorkflow->AlbedoMap());
          }
          if (!pbrWorkflow->NormalMap().empty())
          {
            pbrCommon.SetNormalMap(pbrWorkflow->NormalMap());
          }
          if (!pbrWorkflow->RoughnessMap().empty())
          {
            pbrCommon.SetRoughnessMap(pbrWorkflow->RoughnessMap());
          }
          materialCommon->SetPbrMaterial(pbrCommon);
        }
      }

      materialsSDF.insert(std::pair<std::string, sdf::Material>(materialName, material));
      materials.insert(std::pair<std::string, std::shared_ptr<ignition::common::Material>>
        (materialName, materialCommon));
    }
  }

  // Get all Link elements
  for (auto const &prim : range)
  {
    if (prim.HasAPI<pxr::UsdPhysicsRigidBodyAPI>())
    {
      nameLink = pxr::TfStringify(prim.GetPath());
    }

    if (model->name_.empty() && !prim.IsA<pxr::UsdPhysicsScene>())
      model->name_ = prim.GetName().GetText();

    if (prim.IsA<pxr::UsdShadeMaterial>() || prim.IsA<pxr::UsdShadeShader>())
    {
      continue;
    }

    if (skipNext)
    {
      --skipNext;
      continue;
    }

    std::string primName = pxr::TfStringify(prim.GetPath());

    sdferr << "------------------------------------------------------\n";
    std::cerr << "pathName " << primName << "\n";

    removeSubStr(primName, "/World");

    std::vector<std::string> tokens = ignition::common::split(primName, "/");
    if (tokens.size() == 0)
      continue;

    if (tokens.size() == 1)
    {
      if (!modelName.empty())
      {
        std::cerr << "modelName " << modelName << '\n';
        std::cerr << "prim.GetName().GetText() " << prim.GetName().GetText() << '\n';
        // float a;
        // std::cin >> a;
        if (modelName != prim.GetName().GetText() && !prim.IsA<pxr::UsdPhysicsScene>())
        {

          auto isSameModel = [&](ModelInterfaceSharedPtr _model){ return _model->name_ == model->name_; };

          if (std::find_if(begin(models), end(models), isSameModel) == models.end())
          {
            model = std::make_shared<ModelInterface>();
            model->world_name_ = defaultName;
            model->clear();
            model->name_ = prim.GetName().GetText();
            models.push_back(model);
            modelName = prim.GetName().GetText();
          }
        }
      }
      modelName = prim.GetName().GetText();
    }
    std::cerr << "\tmodelName " << modelName << " " << tokens.size() << '\n';

    if (tokens.size() > 1)
    {
      baseLink = "/" + tokens[0] + "/" + tokens[1];
    }

    if (primName.find(baseLink) == std::string::npos)
    {
      baseLink = "";
    }

    if (prim.IsA<pxr::UsdLuxBoundableLightBase>())
    {
      std::cerr << "Light!" << '\n';
      std::cerr << "\tprimName " << primName << '\n';
      std::cerr << "\tbaseLink " << baseLink << '\n';
      std::cerr << "\tprimName.find(baseLink) == std::string::npos " << (primName.find(baseLink) == std::string::npos) << '\n';

      auto light = ParseLights(prim, metersPerUnit, baseLink);
      if (light)
      {
        if (model->links_.find(baseLink) == model->links_.end())
        {
          model->lights_.insert(std::pair<std::string, std::shared_ptr<sdf::Light>>
            (prim.GetPath().GetName(), light));
        }
        else
        {
            model->links_[baseLink]->lights_.insert(std::pair<std::string, std::shared_ptr<sdf::Light>>
              (prim.GetPath().GetName(), light));
        }
      }
      continue;
    }

    if(prim.IsA<pxr::UsdGeomCamera>())
    {
      if (!baseLink.empty())
      {
        auto sensor = ParseSensors(prim, metersPerUnit, baseLink);
        std::cerr << "baseLink " << baseLink << '\n';
        if(sensor)
        {
          model->links_[baseLink]->sensors_.insert(
            std::pair<std::string, std::shared_ptr<sdf::Sensor>>
              (prim.GetPath().GetName(), sensor));
        }
        std::cerr << "camera!" << '\n';
      }
      continue;
    }

    if (prim.IsA<pxr::UsdPhysicsScene>())
    {
      usd::ParsePhysicsScene(prim);
    }

    if (prim.IsA<pxr::UsdPhysicsCollisionGroup>())
    {
      continue;
    }

    pxr::TfTokenVector schemas = prim.GetAppliedSchemas();
    bool isPhysxVehicleWheelAPI = false;
    for (auto & token : schemas)
    {
      std::cerr << "GetText " << token.GetText() << '\n';
      if (std::string(token.GetText()) == "PhysxVehicleWheelAPI")
      {
        isPhysxVehicleWheelAPI = true;
      }
    }

    if (prim.IsA<pxr::UsdPhysicsJoint>())
    {
      sdferr << "UsdPhysicsJoint" << "\n";

      std::shared_ptr<sdf::Joint> joint = usd::ParseJoints(prim, primName, metersPerUnit);
      if (joint != nullptr)
      {
        model->joints_.insert(make_pair(joint->Name(), joint));
      }

      continue;
    }

    if (isPhysxVehicleWheelAPI)
    {
      std::pair<std::shared_ptr<sdf::Joint>, LinkSharedPtr> result =
        usd::ParseVehicleJoints(prim, primName, metersPerUnit);
      std::shared_ptr<sdf::Joint> joint = result.first;
      LinkSharedPtr link = result.second;
      if (joint != nullptr)
      {
        std::cerr << "Inserted joint " << joint->Name() << '\n';

        model->joints_.insert(make_pair(joint->Name(), joint));
      }
      if (link != nullptr)
      {
        std::cerr << "Insetred link " << primName << '\n';
        model->links_.insert(make_pair(primName, link));
      }
      skipNext++;
      continue;
    }

    if (tokens.size() == 1)
      continue;

    if (!prim.IsA<pxr::UsdGeomGprim>())
    {
      sdferr << "Not a geometry" << "\n";
      continue;
    }

    sdferr << "baseLink " << baseLink << "\n";
    sdferr << "nameLink " << nameLink << "\n";

    LinkSharedPtr link = nullptr;
    auto it = model->links_.find(nameLink);
    if (it != model->links_.end())
    {
      usd::ParseLinks(prim, it->second, materials, materialsSDF, metersPerUnit, skipNext);
    }
    else
    {
      usd::ParseLinks(prim, link, materials, materialsSDF, metersPerUnit, skipNext);

      if (link)
      {
        link->name = nameLink;
        model->links_.insert(make_pair(nameLink, link));
      }
    }

    // if (model->links_.empty()){
    //   model.reset();
    //   return model;
    // }
  }

  for (auto & m : models)
  {
    for (auto & link: m->links_)
    {
      if (ignition::math::equal(link.second->inertial->MassMatrix().Mass(), 0.0))
      {
        if (link.second->name == "world" || link.second->name.empty())
          continue;
        std::cerr << "Added joint to " << link.second->name << '\n';
        ignition::math::MassMatrix3d massMatrix;
        massMatrix.SetMass(0.0001);
        massMatrix.SetDiagonalMoments(
          ignition::math::Vector3d());

        link.second->inertial->SetPose(ignition::math::Pose3d(
            ignition::math::Vector3d(0, 0, 0),
            ignition::math::Quaterniond(1.0, 0, 0, 0)));

        link.second->inertial->SetMassMatrix(massMatrix);
        // std::shared_ptr<sdf::Joint> joint = nullptr;
        // joint = std::make_shared<sdf::Joint>();
        // std::string joint_name = link.second->name;
        // joint->SetName(joint_name + "_joint");
        //
        // joint->SetType(sdf::JointType::FIXED);
        // joint->SetParentLinkName("world");
        // joint->SetChildLinkName(link.second->name);
        // m->joints_.insert(make_pair(joint->Name(), joint));
      }
    }
  }

  std::cerr << "models " << models.size() << '\n';

  for (auto & m : models)
  {
    m->links_.erase("");
  }

  for (auto & m : models)
  {
    std::cerr << "++++++++++++++++++++++++++++++++++++++++++++++++" << '\n';
    std::cerr << "\tmodel name " << m->name_ << '\n';

    for (auto & joint : m->joints_)
    {
      if (joint.second->ParentLinkName() == "world")
      {
        // insert <link name="world"/>
        LinkSharedPtr worldLink = nullptr;
        worldLink.reset(new Link);
        worldLink->clear();
        worldLink->name = "world";
        m->links_.insert(make_pair(worldLink->name, worldLink));
      }
    }

    std::cerr << "\t********************************" << '\n';
    std::cerr << "\tmodel->links_ " << m->links_.size() << '\n';
    for (auto link: m->links_)
    {
      std::cerr << "\tlink name "
                   << "\n\t\t" << link.first
                   << "\n\t\t" << link.second->name << '\n';
    }

    std::cerr << "\t................................." << '\n';
    std::cerr << "\tmodel->joints_ " << m->joints_.size() << '\n';
    for (auto joint: m->joints_)
    {
      std::cerr << "\tjoints name " << joint.second->Name() << '\n';
      std::cerr << "\t\t parent " << joint.second->ParentLinkName() << '\n';
      std::cerr << "\t\t child " << joint.second->ChildLinkName() << '\n';
    }
    std::cerr << "\t********************************" << '\n';

    // every link has children links and joints, but no parents, so we create a
    // local convenience data structure for keeping child->parent relations
    std::map<std::string, std::string> parent_link_tree;
    parent_link_tree.clear();

    // building tree: name mapping
    try
    {
      m->initTree(parent_link_tree);
    }
    catch(ParseError &e)
    {
      m.reset();
      sdferr << "error initTree " << e.what()  << "\n";
      // return model;
    }

    // find the root link
    try
    {
      m->initRoot(parent_link_tree);
    }
    catch(ParseError &e)
    {
      m.reset();
      sdferr << "error initRoot " << e.what() << "\n";
      // return model;
    }
  }

  return models;
}

bool isUSD(const std::string &filename)
{
  auto referencee = pxr::UsdStage::Open(filename);
  return referencee != nullptr;
}

std::vector<ModelInterfaceSharedPtr> parseUSDFile(const std::string &filename)
{
  auto referencee = pxr::UsdStage::Open(filename);
  // std::ifstream stream(filename.c_str());
  if (!referencee)
  {
    return std::vector<ModelInterfaceSharedPtr>();
  }
  // std::string xml_str((std::istreambuf_iterator<char>(stream)),
  //                      std::istreambuf_iterator<char>());
  return usd::parseUSD(filename);
}

void exportUSD()
{

}
}
