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
#include <fstream>

#include <ignition/common/ColladaExporter.hh>
#include <ignition/common/ColladaLoader.hh>
#include <ignition/common/Material.hh>
#include <ignition/common/Mesh.hh>
#include <ignition/common/SubMesh.hh>
#include <ignition/common/Util.hh>

#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/gf/rotation.h>
#include <pxr/base/gf/vec3d.h>
#include <pxr/base/tf/stringUtils.h>

#include <pxr/usd/sdf/reference.h>
#include <pxr/usd/usd/attribute.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/stage.h>

#include <pxr/usd/usdGeom/camera.h>
#include <pxr/usd/usdGeom/gprim.h>

#include <pxr/usd/usdLux/boundableLightBase.h>
#include <pxr/usd/usdLux/nonboundableLightBase.h>
#include <pxr/usd/usdLux/lightAPI.h>
#include <pxr/usd/usdLux/sphereLight.h>

#include <pxr/usd/usdPhysics/collisionGroup.h>
#include <pxr/usd/usdPhysics/fixedJoint.h>
#include <pxr/usd/usdPhysics/joint.h>
#include <pxr/usd/usdPhysics/rigidBodyAPI.h>
#include <pxr/usd/usdPhysics/scene.h>

#include <pxr/usd/usdShade/material.h>
#include <pxr/usd/usdShade/material.h>
#include <pxr/usd/usdShade/materialBindingAPI.h>

#include "sdf/Console.hh"
#include "sdf/Mesh.hh"
#include "sdf/Pbr.hh"

#include "usd_parser/parser_usd.hh"
#include "physics.hh"
#include "joints.hh"
#include "lights.hh"
#include "links.hh"
#include "sensors.hh"
#include "utils.hh"

#include "usd/USDData.hh"

namespace usd {

WorldInterfaceSharedPtr parseUSD(const std::string &xml_string)
{
  usd::USDData usdData(xml_string);
  usdData.Init();
  usdData.ParseMaterials();

  std::cout << usdData << '\n';

  WorldInterfaceSharedPtr world = std::make_shared<WorldInterface>();

  ModelInterfaceSharedPtr model;

  auto referencee = pxr::UsdStage::Open(xml_string);
  if (!referencee)
  {
    return nullptr;
  }

  auto range = pxr::UsdPrimRange::Stage(referencee);

  std::string baseLink;
  std::string nameLink;

  world->_worldName = referencee->GetDefaultPrim().GetName().GetText();

  int skipNext = 0;

  // Get all Link elements
  for (auto const &prim : range)
  {
    // if (prim.HasAPI<pxr::UsdPhysicsRigidBodyAPI>())
    // {
      // nameLink = pxr::TfStringify(prim.GetPath());
  //   // }
  //
  //   if (model->name_.empty() && !prim.IsA<pxr::UsdPhysicsScene>())
  //     model->name_ = prim.GetName().GetText();
  //
    if (skipNext)
    {
      --skipNext;
      continue;
    }

    if (std::string(prim.GetPrimTypeInfo().GetTypeName().GetText()) == "RosDifferentialBase")
    {
      auto leftWheelAttr = prim.GetAttribute(pxr::TfToken("leftWheelJointName"));
      auto rightWheelAttr = prim.GetAttribute(pxr::TfToken("rightWheelJointName"));
      auto wheelBaseAttr = prim.GetAttribute(pxr::TfToken("wheelBase"));
      auto wheelRadiusAttr = prim.GetAttribute(pxr::TfToken("wheelRadius"));

      std::shared_ptr<ROSPluginDiffDrive> plugin = std::make_shared<ROSPluginDiffDrive>();
      plugin->pluginName_ = "RosDifferentialBase";

      std::string leftWheelName;
      std::string rightWheelName;
      float wheelBase;
      float wheelRadius;
      wheelBaseAttr.Get<float>(&wheelBase);
      wheelRadiusAttr.Get<float>(&wheelRadius);
      leftWheelAttr.Get<std::string>(&leftWheelName);
      rightWheelAttr.Get<std::string>(&rightWheelName);
      plugin->leftWheelJointName_ = "/" + model->name_ + "/chassis_link/" + leftWheelName + "_joint";
      plugin->rightWheelJointName_ = "/" + model->name_ + "/chassis_link/" + leftWheelName + "_joint";
      plugin->wheelBase_ = wheelBase;
      plugin->wheelRadius_ = wheelRadius;

      model->plugins_.push_back(plugin);
    }

    if (prim.IsA<pxr::UsdPhysicsScene>())
    {
      usd::ParsePhysicsScene(prim, world);
      continue;
    }

    if (prim.IsA<pxr::UsdShadeMaterial>() || prim.IsA<pxr::UsdShadeShader>())
    {
      continue;
    }

    std::string primName = pxr::TfStringify(prim.GetPath());
    sdferr << "------------------------------------------------------\n";
    std::cerr << "pathName " << primName << "\n";
  //
  //   removeSubStr(primName, "/World");
  //
    std::vector<std::string> tokens = ignition::common::split(primName, "/");
    if (tokens.size() == 0)
      continue;

    if (tokens.size() == 1)
    {
      model = std::make_shared<ModelInterface>();
      model->clear();
      model->name_ = tokens[0];
      world->_models.push_back(model);

      double metersPerUnit;
      referencee->GetMetadata<double>(
        pxr::TfToken("metersPerUnit"), &metersPerUnit);
      ignition::math::Pose3d pose;
      ignition::math::Vector3d scale{1, 1, 1};

      GetTransform(
        prim,
        usdData,
        pose,
        scale,
        model->name_);
      model->pose = pose;
    }

    if (tokens.size() >=2)
    {
      bool shortName = false;
      if (tokens.size() == 2)
      {
        if (prim.IsA<pxr::UsdGeomGprim>() ||
            (std::string(prim.GetPrimTypeInfo().GetTypeName().GetText()) == "Plane"))
        {
          if (prim.GetPath().GetName() != "imu")
          {
            nameLink = "/" + tokens[0];
            shortName = true;
          }
        }
      }
      if(!shortName)
      {
        nameLink = "/" + tokens[0] + "/" + tokens[1];
      }
    }

    if (tokens.size() > 1)
    {
      bool shortName = false;
      if (tokens.size() == 2)
      {
        if (prim.IsA<pxr::UsdGeomGprim>() ||
            (std::string(prim.GetPrimTypeInfo().GetTypeName().GetText()) == "Plane"))
        {
          if (prim.GetPath().GetName() != "imu")
          {
            baseLink = "/" + tokens[0];
            shortName = true;
            // exit(-1);
          }
        }
      }
      if(!shortName)
      {
        baseLink = "/" + tokens[0] + "/" + tokens[1];
      }
    }

    if (primName.find(baseLink) == std::string::npos)
    {
      baseLink = "";
    }

    if (prim.IsA<pxr::UsdLuxBoundableLightBase>() || prim.IsA<pxr::UsdLuxNonboundableLightBase>())
    {

      if (tokens.size() == 1)
      {
        baseLink = tokens[0];
      }

      std::cerr << "Light!" << '\n';
      std::cerr << "\tprimName " << primName << '\n';
      std::cerr << "\tbaseLink " << baseLink << '\n';
      std::cerr << "\tprimName.find(baseLink) == std::string::npos " << (primName.find(baseLink) == std::string::npos) << '\n';

      auto light = ParseLights(prim, usdData, baseLink);
      if (light)
      {
        std::cerr << "\tLight parsed" << '\n';
        if (model->links_.find(baseLink) == model->links_.end())
        {
          std::cerr << "\tworld->_lights" << '\n';
          world->_lights.insert(std::pair<std::string, std::shared_ptr<sdf::Light>>
            (prim.GetPath().GetName(), light));
        }
        else
        {
           std::cerr << "\tmodel->_lights" << '\n';
           model->links_[baseLink]->lights_.insert(std::pair<std::string, std::shared_ptr<sdf::Light>>
              (prim.GetPath().GetName(), light));
        }
      }
      continue;
    }
    if(prim.IsA<pxr::UsdGeomCamera>() ||
       std::string(prim.GetPrimTypeInfo().GetTypeName().GetText()) == "Lidar")
    {
      if (!baseLink.empty())
      {
        auto sensor = ParseSensors(prim, usdData, baseLink);
        std::cerr << "baseLink " << baseLink << '\n';
        if(sensor)
        {
          auto it = model->links_.find(baseLink);
          if (it != model->links_.end())
          {
            model->links_[baseLink]->sensors_.insert(
              std::pair<std::string, std::shared_ptr<sdf::Sensor>>
                (prim.GetPath().GetName(), sensor));
          }
          else
          {
            sdferr << "Not able to insert sensor " << baseLink << " is missing" << "\n";
            for (auto & link : model->links_)
            {
              sdferr << "link names " << link.first << " " << link.second->name << "\n";
            }
          }
        }
        std::cerr << "camera!" << '\n';
      }
      continue;
    }


  //
  //   if (prim.IsA<pxr::UsdPhysicsCollisionGroup>())
  //   {
  //     continue;
  //   }
  //
  //   pxr::TfTokenVector schemas = prim.GetAppliedSchemas();
  //   bool isPhysxVehicleWheelAPI = false;
  //   for (auto & token : schemas)
  //   {
  //     std::cerr << "GetText " << token.GetText() << '\n';
  //     if (std::string(token.GetText()) == "PhysxVehicleWheelAPI")
  //     {
  //       isPhysxVehicleWheelAPI = true;
  //     }
  //   }
  //
    if (prim.IsA<pxr::UsdPhysicsJoint>())
    {
      sdferr << "UsdPhysicsJoint" << "\n";

      std::shared_ptr<sdf::Joint> joint =
        usd::ParseJoints(prim, primName, usdData);
      if (joint != nullptr)
      {
        model->joints_.insert(make_pair(joint->Name(), joint));
      }

      continue;
    }
  //
  //   if (isPhysxVehicleWheelAPI)
  //   {
  //     std::pair<std::shared_ptr<sdf::Joint>, LinkSharedPtr> result =
  //       usd::ParseVehicleJoints(prim, primName, metersPerUnit);
  //     std::shared_ptr<sdf::Joint> joint = result.first;
  //     LinkSharedPtr link = result.second;
  //     if (joint != nullptr)
  //     {
  //       std::cerr << "Inserted joint " << joint->Name() << '\n';
  //
  //       model->joints_.insert(make_pair(joint->Name(), joint));
  //     }
  //     if (link != nullptr)
  //     {
  //       std::cerr << "Insetred link " << primName << '\n';
  //       model->links_.insert(make_pair(primName, link));
  //     }
  //     skipNext++;
  //     continue;
  //   }
  //
  //   if (tokens.size() == 1)
  //     continue;
  //
    if (!prim.IsA<pxr::UsdGeomGprim>() && !(std::string(prim.GetPrimTypeInfo().GetTypeName().GetText()) == "Plane"))
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
      usd::ParseLinks(prim, nameLink, it->second, usdData, skipNext);
    }
    else
    {
      usd::ParseLinks(prim, nameLink, link, usdData, skipNext);

      if (link)
      {
        model->links_.insert(make_pair(nameLink, link));
      }
    }
  }

  for (auto & m : world->_models)
  {
    for (auto & link: m->links_)
    {
      if (ignition::math::equal(link.second->inertial->MassMatrix().Mass(), 0.0) ||
          link.second->inertial->MassMatrix().DiagonalMoments() == ignition::math::Vector3d(0., 0., 0.))
      {
        if (link.second->name == "world" || link.second->name.empty())
          continue;
        std::cerr << "Added joint to " << link.second->name << '\n';
        ignition::math::MassMatrix3d massMatrix = link.second->inertial->MassMatrix();
        if (ignition::math::equal(link.second->inertial->MassMatrix().Mass(), 0.0))
        {
          massMatrix.SetMass(0.0001);
        }
        if (link.second->inertial->MassMatrix().DiagonalMoments() == ignition::math::Vector3d(0., 0., 0.))
        {
          massMatrix.SetDiagonalMoments(
            ignition::math::Vector3d(0.0001, 0.0001, 0.0001));
        }

        link.second->inertial->SetPose(ignition::math::Pose3d(
            ignition::math::Vector3d(0, 0, 0),
            ignition::math::Quaterniond(1.0, 0, 0, 0)));

        link.second->inertial->SetMassMatrix(massMatrix);

        bool linkHasJoint = false;
        for (auto & joint: m->joints_)
        {
          if (joint.second->ParentLinkName() == link.first ||
              joint.second->ChildLinkName() == link.first)
          {
            linkHasJoint = true;
          }
        }
        if (!linkHasJoint)
        {
          std::shared_ptr<sdf::Joint> joint = nullptr;
          joint = std::make_shared<sdf::Joint>();
          std::string joint_name = link.second->name;
          joint->SetName(joint_name + "_joint");

          joint->SetType(sdf::JointType::FIXED);
          joint->SetParentLinkName("world");
          joint->SetChildLinkName(link.second->name);
          m->joints_.insert(make_pair(joint->Name(), joint));
        }
      }
    }
  }

  for(auto iteratorModel = world->_models.begin(); iteratorModel != world->_models.end();)
  {
    auto & m = *iteratorModel;
    // if (!m)
    //   continue;

    if (m->links_.size() == 0)
    {
       std::cerr << "deleting Empty Model: " << m->name_ << '\n';
       world->_models.erase(iteratorModel);
       continue;
     }

    std::cerr << "++++++++++++++++++++++++++++++++++++++++++++++++" << '\n';
    std::cerr << "\tmodel name " << m->name_ << '\n';

    if (m->links_.size() == 1)
    {
      std::shared_ptr<sdf::Joint> joint = nullptr;
      joint = std::make_shared<sdf::Joint>();
      auto it = m->links_.begin();
      std::string joint_name = it->second->name;
      joint->SetName(joint_name + "_joint");

      joint->SetType(sdf::JointType::FIXED);
      joint->SetParentLinkName("world");
      joint->SetChildLinkName(it->second->name);
      m->joints_.insert(make_pair(joint->Name(), joint));
    }

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
                   // << "\n\t\t" << link.first
                   << "\n\t\t" << link.second->name << '\n';

     for (auto visual: link.second->visual_array)
     {
       std::cerr << "\t\tvisual name "
                  << "\n\t\t\t" << visual->Name() << "\n";
     }
     for (auto col: link.second->collision_array)
     {
       std::cerr << "\t\tCollision name "
                  << "\n\t\t\t" << col->Name() << "\n";
     }
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
      // continue;
    }

    try
    {
      // find the root link
      m->initRoot(parent_link_tree);
    }
    catch(ParseError &e)
    {
      m.reset();
      sdferr << "error initRoot " << e.what() << "\n";
    }

    ++iteratorModel;
  }
  //
  // return models;
  return world;
}

bool isUSD(const std::string &filename)
{
  auto referencee = pxr::UsdStage::Open(filename);
  return referencee != nullptr;
}

WorldInterfaceSharedPtr parseUSDFile(const std::string &filename)
{
  auto referencee = pxr::UsdStage::Open(filename);
  // std::ifstream stream(filename.c_str());
  if (!referencee)
  {
    return nullptr;
  }
  // std::string xml_str((std::istreambuf_iterator<char>(stream)),
  //                      std::istreambuf_iterator<char>());
  return usd::parseUSD(filename);
}

void exportUSD()
{

}
}
