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

#ifndef SDF_USD_PARSER_USDDATA_HH
#define SDF_USD_PARSER_USDDATA_HH

#include <memory>
#include <set>
#include <string>
#include <utility>
#include <unordered_map>

// TODO(ahcorde):this is to remove deprecated "warnings" in usd, these warnings
// are reported using #pragma message so normal diagnostic flags cannot remove
// them. This workaround requires this block to be used whenever usd is
// included.
#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <ignition/utils/ImplPtr.hh>
#pragma pop_macro ("__DEPRECATED")

#include "sdf/Material.hh"
#include "sdf/sdf_config.h"
#include "sdf/system_util.hh"
#include "sdf/usd_parser/USDStage.hh"

namespace sdf
{
  // Inline bracke to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  namespace usd
  {
    /// \brief this class will handle the data of a stage
    /// It will parse the materials in the stage
    /// If the stage has some references to another stages, this class
    /// will read them and make this data available here too.
    class SDFORMAT_VISIBLE USDData
    {
      /// \brief Constructor
      public: USDData(const std::string &_filename);

      /// \brief Initialize the data inside the class with the stage
      /// defined in the consttuctor
      /// \return True if initialization was successful or False otherwise
      public: bool Init();

      /// \brief If a stage contains substages, this will allow to include
      /// them.
      /// \return True if the stage was added successfully or False otherwise
      public: bool AddStage(const std::string &_ref);

      /// \brief Read materials
      /// \return Number of materials readed
      public: int ParseMaterials();

      /// \brief Get all materials readed in the stage
      public: const std::unordered_map<std::string, sdf::Material> &
        GetMaterials() const;

      /// \brief Get all sdformat models inside the stage
      public: const std::set<std::string> &GetModels() const;

      /// \brief Get all stages (the main one and all the referenced).
      public: const std::unordered_map<std::string, std::shared_ptr<USDStage>> &
        GetAllReferences() const;

      /// \brief Find a path and get the data
      /// \param[in] _name Name of the path to find
      /// \return A paair with the name of the stage and the data
      public: const std::pair<std::string, std::shared_ptr<sdf::usd::USDStage>>
          findStage(const std::string &_name);

      friend std::ostream& operator<<(std::ostream& os, const USDData& data)
      {
        os << "References:" << "\n";
        for (auto &ref : data.GetAllReferences())
        {
          os << "\t" << ref.first << "\n";
          // os << "\t\t" << ref.second->_referenceName << "\n";
          os << "\t\t" << ref.second->GetUpAxis() << "\n";
          os << "\t\t" << ref.second->GetMetersPerUnit() << "\n";
          for (auto &path : ref.second->GetUSDPaths())
          {
            std::cerr << "\t\tPath " << path << '\n';
          }
        }
        os << "Models:" << "\n";
        auto models = data.GetModels();
        for (const auto &model : models)
        {
          os << "\t" << model << "\n";
        }

        return os;
      }

      /// \brief Private data pointer.
      IGN_UTILS_IMPL_PTR(dataPtr)
    };
  }
}
}

#endif
