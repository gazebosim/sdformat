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

#include "pxr/usd/usdLux/light.h"
#include "pxr/usd/usdLux/sphereLight.h"

// #include "pxr/base/tf/staticTokens.h"
// #include "pxr/usd/usdRi/tokens.h"

#include "usd_parser/parser_usd.hh"
#include "physics.hh"
#include "joints.hh"
#include "links.hh"
#include "utils.hh"
#include "sdf/Console.hh"
#include "sdf/Mesh.hh"

#include <fstream>

#include <ignition/common/Util.hh>
#include <ignition/common/Mesh.hh>
#include <ignition/common/SubMesh.hh>
#include <ignition/common/ColladaLoader.hh>
#include <ignition/common/ColladaExporter.hh>
#include <ignition/common/Material.hh>

namespace usd {

ModelInterfaceSharedPtr parseUSD(const std::string &xml_string)
{
  ModelInterfaceSharedPtr model(new ModelInterface);
  model->clear();

  auto referencee = pxr::UsdStage::CreateInMemory();
  if (!referencee->GetRootLayer()->ImportFromString(xml_string))
  {
    return nullptr;
  }

  // Get robot name
  const char *name = "test";
  if (!name)
  {
    model.reset();
    return model;
  }
  model->name_ = std::string(name);

  // TODO(ahcorde): Get all Material elements

  auto range = pxr::UsdPrimRange::Stage(referencee);

  double metersPerUnit;
  referencee->GetMetadata<double>(pxr::TfToken("metersPerUnit"), &metersPerUnit);
  std::cerr << "/* metersPerUnit */" << metersPerUnit << '\n';
  std::string rootPath;
  std::string nameLink;

  int skipNext = 0;

  std::map<std::string, std::shared_ptr<ignition::common::Material>> materials;

  // Get all Link elements
  for (auto const &prim : range) {

    if (skipNext)
    {
      --skipNext;
      continue;
    }

    std::string primName = pxr::TfStringify(prim.GetPath());

    sdferr << "------------------------------------------------------\n";
    sdferr << "pathName " << primName << "\n";
    // sdferr << prim.GetName().GetText() << "\n";

    size_t pos = std::string::npos;
    if ((pos  = primName.find("/World") )!= std::string::npos)
    {
      primName.erase(pos, std::string("/World").length());
    }

    std::vector<std::string> tokens = ignition::common::split(primName, "/");
    if (tokens.size() == 0)
      continue;

    if (tokens.size() == 1)
    {
      rootPath = prim.GetName().GetText();
    }
    std::cerr << "rootPath " << rootPath << " " << tokens.size() << '\n';

    if (tokens.size() > 1)
    {
      nameLink = "/" + tokens[0] + "/" + tokens[1];
    }

    if (prim.IsA<pxr::UsdPhysicsScene>())
    {
      usd::ParsePhysicsScene(prim);
    }

    if (prim.IsA<pxr::UsdPhysicsCollisionGroup>())
    {
      continue;
    }

    if (prim.IsA<pxr::UsdShadeMaterial>())
    {
      auto variantMaterial = pxr::UsdShadeMaterial(prim);

      std::cerr << "Material!!! " << variantMaterial.GetPath().GetName() << '\n';
      sdf::Material material = ParseMaterial(prim, skipNext);
      std::string materialName = std::string(variantMaterial.GetPath().GetName());

      std::shared_ptr<ignition::common::Material> materialCommon
         = std::make_shared<ignition::common::Material>();

      materialCommon->SetEmissive(material.Emissive());
      materialCommon->SetDiffuse(material.Diffuse());

      materials.insert(std::pair<std::string, std::shared_ptr<ignition::common::Material>>
        (materialName, materialCommon));
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

    if (prim.IsA<pxr::UsdLuxLight>())
    {
      sdferr << "Light" << "\n";
      if (prim.IsA<pxr::UsdLuxSphereLight>())
      {
        sdferr << "Sphere light" << "\n";
      }
      continue;
    }

    if (!prim.IsA<pxr::UsdGeomGprim>())
    {
      sdferr << "Not a geometry" << "\n";
      continue;
    }

    sdferr << "nameLink " << nameLink << "\n";

    LinkSharedPtr link = nullptr;
    auto it = model->links_.find(nameLink);
    if (it != model->links_.end())
    {
      link = usd::ParseLinks(prim, it->second, metersPerUnit, skipNext);
    }
    else
    {
      link = usd::ParseLinks(prim, link, metersPerUnit, skipNext);

      if (link)
      {
          model->links_.insert(make_pair(nameLink, link));
      }
    }

    if (model->links_.empty()){
      model.reset();
      return model;
    }
  }

  for (auto & joint : model->joints_)
  {
    if (joint.second->ParentLinkName() == "world")
    {
      // insert <link name="world"/>
      LinkSharedPtr worldLink = nullptr;
      worldLink.reset(new Link);
      worldLink->clear();
      worldLink->name = "world";
      model->links_.insert(make_pair(worldLink->name, worldLink));
    }
  }

  std::cerr << "++++++++++++++++++++++++++++++++++++++++++" << '\n';
  for (auto &link: model->links_)
  {
    std::cerr << ">>>>>> link->name " << link.second->name << '\n';
    for (unsigned int i = 0; i < link.second->visual_array.size(); ++i)
    {
      std::string materialName = link.second->visual_array_material_name[i];
      if (!materialName.empty())
      {
        std::cerr << "materialName " << materialName << '\n';

        auto it = materials.find(materialName);
        if (it != materials.end())
        {
          if (link.second->visual_array[i]->Geom() != nullptr)
          {
            if (link.second->visual_array[i]->Geom()->Type() == sdf::GeometryType::MESH)
            {
              ignition::common::ColladaLoader colladaLoader;
              std::string fileName = link.second->visual_array[i]->Geom()->MeshShape()->FilePath();
              std::cerr << "fileName " << fileName << '\n';
              // int a;
              // std::cin >> a;
              ignition::common::Mesh * meshReload = colladaLoader.Load(fileName);
              if (meshReload)
              {
                std::cerr << "meshReload " << meshReload->SubMeshCount() << '\n';
                std::cerr << "meshReload " << meshReload->VertexCount() << '\n';
                for (unsigned j = 0; j < meshReload->SubMeshCount(); j++)
                {
                  std::weak_ptr<ignition::common::SubMesh> subMesh = meshReload->SubMeshByIndex(j);
                  std::cerr << "subMesh Name " << subMesh.lock()->Name() << '\n';
                  std::cerr << "link.second->visual_array[i] Name " << link.second->visual_array[i]->Name() << '\n';

                  int index = meshReload->AddMaterial(it->second);
                  subMesh.lock()->SetMaterialIndex(index);
                }

                ignition::common::ColladaExporter exporter;
                exporter.Export(meshReload, fileName.substr(0, fileName.size() - 4), false);
                delete meshReload;
              }
            }
          }
        }
      }
    }
  }
  std::cerr << "++++++++++++++++++++++++++++++++++++++++++" << '\n';


  std::cerr << "********************************" << '\n';
  std::cerr << "model->links_ " << model->links_.size() << '\n';
  for (auto link: model->links_)
  {
    std::cerr << "link name " << link.second->name << '\n';
  }

  std::cerr << "................................." << '\n';
  std::cerr << "model->joints_ " << model->joints_.size() << '\n';
  for (auto joint: model->joints_)
  {
    std::cerr << "joints name " << joint.second->Name() << '\n';
    std::cerr << "\t parent " << joint.second->ParentLinkName() << '\n';
    std::cerr << "\t child " << joint.second->ChildLinkName() << '\n';
  }
  std::cerr << "********************************" << '\n';


  // every link has children links and joints, but no parents, so we create a
  // local convenience data structure for keeping child->parent relations
  std::map<std::string, std::string> parent_link_tree;
  parent_link_tree.clear();

  // building tree: name mapping
  try
  {
    model->initTree(parent_link_tree);
  }
  catch(ParseError &e)
  {
    model.reset();
    sdferr << "error initTree " << e.what()  << "\n";
    return model;
  }

  // find the root link
  try
  {
    model->initRoot(parent_link_tree);
  }
  catch(ParseError &e)
  {
    model.reset();
    sdferr << "error initRoot " << e.what() << "\n";
    return model;
  }

  if (referencee)
    return model;
  return nullptr;
}

bool isUSD(const std::string &filename)
{
  auto referencee = pxr::UsdStage::Open(filename);
  return referencee != nullptr;
}

ModelInterfaceSharedPtr parseUSDFile(const std::string &filename)
{
  auto referencee = pxr::UsdStage::Open(filename);
  std::ifstream stream( filename.c_str() );
  if (!stream)
  {
    return ModelInterfaceSharedPtr();
  }
  std::string xml_str((std::istreambuf_iterator<char>(stream)),
                     std::istreambuf_iterator<char>());
  return usd::parseUSD(xml_str);
}

void exportUSD()
{

}
}
