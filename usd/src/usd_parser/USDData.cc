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
#include "sdf/usd/usd_parser/USDData.hh"

#include <set>
#include <string>
#include <unordered_map>

// TODO(ahcorde) this is to remove deprecated "warnings" in usd, these warnings
// are reported using #pragma message so normal diagnostic flags cannot remove
// them. This workaround requires this block to be used whenever usd is
// included.
#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/usd/usd/primCompositionQuery.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usdPhysics/scene.h>
#include <pxr/usd/usdShade/material.h>
#pragma pop_macro ("__DEPRECATED")

#include <gz/common/Filesystem.hh>
#include <gz/common/Util.hh>

#include "sdf/Material.hh"
#include "USDMaterial.hh"

namespace sdf {
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
//
namespace usd {
  /// \brief USDStage private data.
  class USDData::Implementation
  {
    public:
      /// File name of the main stage
      std::string filename;

      /// Directory where the main stage is located
      std::string directoryPath;

      /// map with the filename and its own Stage data
      std::unordered_map<std::string, std::shared_ptr<USDStage>> references;

      /// Models
      std::set<std::string> models;

      /// Materials availables in the stage and substages
      std::unordered_map<std::string, sdf::Material> materials;

      /// Add all subdirectories that are inside the stage folder
      /// This will help us to find other stages and/or textures.
      /// \param[in] _path Path of the subdirectory to add
      void AddSubdirectories(const std::string &_path)
      {
        for (gz::common::DirIter file(_path);
          file != gz::common::DirIter(); ++file)
        {
          std::string current(*file);

          if (gz::common::isDirectory(current))
          {
            auto systemPaths = gz::common::systemPaths();
            systemPaths->AddFilePaths(current);
            this->AddSubdirectories(current);
          }
        }
      }
  };

  /////////////////////////////////////////////////
  USDData::USDData(const std::string &_filename)
    : dataPtr(gz::utils::MakeImpl<Implementation>())
  {
    this->dataPtr->filename = _filename;
  }

  /////////////////////////////////////////////////
  const std::unordered_map<std::string, sdf::Material> &
    USDData::Materials() const
  {
    return this->dataPtr->materials;
  }

  /////////////////////////////////////////////////
  const std::set<std::string> &USDData::Models() const
  {
    return this->dataPtr->models;
  }

  /////////////////////////////////////////////////
  const std::unordered_map<std::string, std::shared_ptr<USDStage>> &
    USDData::AllReferences() const
  {
    return this->dataPtr->references;
  }

  /////////////////////////////////////////////////
  UsdErrors USDData::Init()
  {
    UsdErrors errors;

    auto usdStage = std::make_shared<USDStage>(this->dataPtr->filename);
    UsdErrors errorsInit = usdStage->Init();
    if(!errorsInit.empty())
    {
      errors.insert(errors.end(), errorsInit.begin(), errorsInit.end());
      return errors;
    }

    // Add the base stage to the structure
    this->dataPtr->references.insert(
      {this->dataPtr->filename,
       usdStage
      });

    // it's the stage available
    auto referencee = pxr::UsdStage::Open(this->dataPtr->filename);
    if (!referencee)
    {
      errors.emplace_back(UsdError(
        sdf::usd::UsdErrorCode::INVALID_USD_FILE,
        "Failed to load usd file"));
      return errors;
    }

    this->dataPtr->directoryPath = gz::common::absPath(
      this->dataPtr->filename);

    std::string basename = gz::common::basename(
      this->dataPtr->directoryPath);
    this->dataPtr->directoryPath = gz::common::replaceAll(
        this->dataPtr->directoryPath, basename, "");

    this->dataPtr->AddSubdirectories(this->dataPtr->directoryPath);

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

    return errors;
  }

  /////////////////////////////////////////////////
  UsdErrors USDData::ParseMaterials()
  {
    UsdErrors errors;
    auto referencee = pxr::UsdStage::Open(this->dataPtr->filename);
    if (!referencee)
    {
      errors.emplace_back(UsdError(
        sdf::usd::UsdErrorCode::INVALID_USD_FILE,
        "Failed to load usd file"));
      return errors;
    }

    auto range = pxr::UsdPrimRange::Stage(referencee);

    for (auto const &prim : range)
    {
      std::string primName = pxr::TfStringify(prim.GetPath());
      if (prim.IsA<pxr::UsdShadeMaterial>())
      {
        std::string materialName = prim.GetName();

        if (this->dataPtr->materials.find(materialName)
            != this->dataPtr->materials.end())
        {
          continue;
        }

        sdf::Material material;
        UsdErrors errrosMaterial = ParseMaterial(prim, material);
        if (!errrosMaterial.empty())
        {
          errors.emplace_back(UsdError(
            sdf::usd::UsdErrorCode::SDF_TO_USD_PARSING_ERROR,
            "Error parsing material"));
          errors.insert(
            errors.end(), errrosMaterial.begin(), errrosMaterial.end());
          return errors;
        }
        this->dataPtr->materials.insert(std::pair<std::string, sdf::Material>(
          materialName, material));
      }
    }
    return errors;
  }

  /////////////////////////////////////////////////
  const std::pair<std::string, std::shared_ptr<USDStage>>
    USDData::FindStage(const std::string &_name) const
  {
    for (auto &ref : this->dataPtr->references)
    {
      if (ref.second != nullptr)
      {
        for (auto &path : ref.second->USDPaths())
        {
          if (path == _name)
          {
            return ref;
          }
        }
      }
    }
    return std::make_pair("", nullptr);
  }

  /////////////////////////////////////////////////
  UsdErrors USDData::AddStage(const std::string &_ref)
  {
    UsdErrors errors;
    std::string key = _ref;

    auto search = this->dataPtr->references.find(_ref);
    if (search == this->dataPtr->references.end())
    {
      std::string basename = gz::common::basename(key);
      auto subDirectory = gz::common::replaceAll(key, basename, "");

      auto systemPaths = gz::common::systemPaths();
      systemPaths->AddFilePaths(gz::common::joinPaths(
        this->dataPtr->directoryPath, subDirectory));

      this->dataPtr->AddSubdirectories(gz::common::joinPaths(
        this->dataPtr->directoryPath, subDirectory));

      std::string fileNameRef = gz::common::findFile(basename);
      if (fileNameRef.empty())
      {
        errors.emplace_back(UsdError(
          sdf::usd::UsdErrorCode::SDF_TO_USD_PARSING_ERROR,
          "Not able to find asset [" + _ref + "]"));
        return errors;
      }

      auto usdStage = std::make_shared<USDStage>(fileNameRef);
      UsdErrors errorsInit = usdStage->Init();
      if(!errorsInit.empty())
      {
        errors.insert(errors.end(), errorsInit.begin(), errorsInit.end());
        return errors;
      }

      this->dataPtr->references.insert(
        {key,
         std::make_shared<USDStage>(fileNameRef)
        });
        return errors;
    }
    else
    {
      errors.emplace_back(UsdError(
        sdf::usd::UsdErrorCode::SDF_TO_USD_PARSING_ERROR,
        "Element already exists"));
      return errors;
    }
  }
}
}
}
