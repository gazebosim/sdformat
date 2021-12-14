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

#include "USDData.hh"

#include <filesystem>
#include <iostream>

#include <pxr/usd/usd/primCompositionQuery.h>
#include <pxr/usd/usd/primRange.h>

#include <pxr/usd/usdPhysics/scene.h>

#include <pxr/usd/usdShade/material.h>

#include <ignition/common/Filesystem.hh>
#include <ignition/common/Util.hh>

#include "usd_parser/utils.hh"

namespace usd {
  USDData::USDData(const std::string &_filename)
  {
    this->filename = _filename;

    this->_references.insert(
      {_filename,
       std::make_shared<USDStage>(_filename)
      });
  }

  void addSubdirectories(std::string _path)
  {
    for (ignition::common::DirIter file(_path);
      file != ignition::common::DirIter(); ++file)
    {
      std::string current(*file);

      if (ignition::common::isDirectory(current))
      {
        auto systemPaths = ignition::common::systemPaths();
        systemPaths->AddFilePaths(current);
        addSubdirectories(current);
      }
    }
  }

  bool USDData::Init()
  {
    auto referencee = pxr::UsdStage::Open(this->filename);
    if (!referencee)
    {
      return false;
    }

    std::string basename = ignition::common::basename(this->filename);
    std::string inputPath = removeSubStr(this->filename, basename);
    if (std::filesystem::path(this->filename).is_absolute())
    {
      this->directoryPath = inputPath;
    }
    else
    {
      this->directoryPath = ignition::common::joinPaths(ignition::common::cwd(), inputPath);
    }

    addSubdirectories(directoryPath);

    auto range = pxr::UsdPrimRange::Stage(referencee);

    for (auto const &_prim : range)
    {
      if (_prim.IsA<pxr::UsdShadeMaterial>())
      {
        continue;
      }

      if (_prim.IsA<pxr::UsdPhysicsScene>())
      {
        continue;
      }

      std::string primName = pxr::TfStringify(_prim.GetPath());
      std::vector<std::string> tokens = ignition::common::split(primName, "/");
      if (tokens.size() == 1)
      {
        _models.insert(tokens[0]);
      }

      pxr::UsdPrimCompositionQuery query =
        pxr::UsdPrimCompositionQuery::GetDirectReferences(_prim);

      std::vector<pxr::UsdPrimCompositionQueryArc> arcs =
        query.GetCompositionArcs();
      for (auto & a : arcs )
      {
        pxr::SdfLayerHandle handler = a.GetIntroducingLayer();

        for (auto & ref : handler->GetCompositionAssetDependencies())
        {
          this->AddStage(ref);
        }
      }
    }

    return true;
  }

  int USDData::ParseMaterials()
  {
    auto referencee = pxr::UsdStage::Open(this->filename);
    if (!referencee)
    {
      return false;
    }

    auto range = pxr::UsdPrimRange::Stage(referencee);

    for (auto const &prim : range)
    {
      std::string primName = pxr::TfStringify(prim.GetPath());
      if (prim.IsA<pxr::UsdShadeMaterial>())
      {
        std::string materialName = prim.GetName();

        if (materials.find(materialName) != materials.end())
        {
          continue;
        }

        std::cerr << "New material!!! " << materialName << '\n';
        sdf::Material material = ParseMaterial(prim);
        materials.insert(std::pair<std::string, sdf::Material>(materialName, material));
      }
    }
    return materials.size();
  }

  std::pair<std::string, std::shared_ptr<USDStage>> USDData::findStage(const std::string &_name)
  {
    for (auto &ref : this->_references)
    {
      if (ref.second != nullptr)
      {
        for (auto &path : ref.second->_paths)
        {
          if (path == _name)
          {
            return ref;
          }
        }
      }
    }
    return std::make_pair<std::string, std::shared_ptr<USDStage>>("", nullptr);
  }

  bool USDData::AddStage(const std::string &_ref)
  {
    std::string key = _ref;

    auto search = this->_references.find(_ref);
    if (search == this->_references.end())
    {
      std::string basename = ignition::common::basename(key);
      std::string subDirectory = removeSubStr(key, basename);

      auto systemPaths = ignition::common::systemPaths();
      std::cerr << "subDirectory " << ignition::common::joinPaths(this->directoryPath, subDirectory) << '\n';
      systemPaths->AddFilePaths(ignition::common::joinPaths(this->directoryPath, subDirectory));

      addSubdirectories(ignition::common::joinPaths(this->directoryPath, subDirectory));

      std::string fileNameRef = ignition::common::findFile(basename);
      if (fileNameRef.empty())
      {
        std::cerr << "Not able to find asset [" << _ref << "]" << '\n';
        return false;
      }

      // Add
      this->_references.insert(
        {key,
         std::make_shared<USDStage>(fileNameRef)
        });
      std::cout << "inserted " << key << '\n';
      return true;
    }
    return false;
  }

  std::ostream& operator<<(std::ostream& os, const USDData& data)
  {
    os << "References:" << "\n";
    for (auto &ref : data._references)
    {
      os << "\t" << ref.first << "\n";
      os << "\t\t" << ref.second->_referenceName << "\n";
      os << "\t\t" << ref.second->_upAxis << "\n";
      os << "\t\t" << ref.second->_metersPerUnit << "\n";
      for (auto &path : ref.second->_paths)
      {
        std::cerr << "\t\tPath " << path << '\n';
      }
    }
    os << "Models:" << "\n";
    for (auto &model : data._models)
    {
      os << "\t" << model << "\n";
    }

    return os;
  }
}
