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

#ifndef SDF_USD_USD_PARSER_USDDATA_HH_
#define SDF_USD_USD_PARSER_USDDATA_HH_

#include <memory>
#include <set>
#include <string>
#include <unordered_map>
#include <utility>

#include <ignition/utils/ImplPtr.hh>

#include "sdf/Material.hh"
#include "sdf/Types.hh"
#include "sdf/sdf_config.h"
#include "sdf/system_util.hh"
#include "sdf/usd/usd_parser/USDStage.hh"
#include "sdf/usd/Export.hh"
#include "sdf/usd/UsdError.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  namespace usd
  {
    /// \brief this class will handle the data of a stage
    /// It will parse the materials in the stage
    /// If the stage has some references to other stages, this class
    /// will read them and make this data available here too.
    class IGNITION_SDFORMAT_USD_VISIBLE USDData
    {
      /// \brief Constructor
      public: explicit USDData(const std::string &_filename);

      /// \brief Initialize the data inside the class with the stage
      /// defined in the constructor
      /// \return A vector of Error objects. Each Error includes
      /// an error code and message. An empty vector indicates no error.
      public: UsdErrors Init();

      /// \brief If a stage contains substages, this will allow to include
      /// them.
      /// \return A vector of Error objects. Each Error includes
      /// an error code and message. An empty vector indicates no error.
      public: UsdErrors AddStage(const std::string &_ref);

      /// \brief Read materials
      /// \return A vector of Error objects. Each Error includes
      /// an error code and message. An empty vector indicates no error.
      public: UsdErrors ParseMaterials();

      /// \brief Get all materials readed in the stage
      public: const std::unordered_map<std::string, sdf::Material> &
        Materials() const;

      /// \brief Get all sdformat models inside the stage
      public: const std::set<std::string> &Models() const;

      /// \brief Get all stages (the main one and all the referenced).
      public: const std::unordered_map<std::string, std::shared_ptr<USDStage>> &
        AllReferences() const;

      /// \brief Find a path and get the data
      /// \param[in] _name Name of the path to find
      /// \return A pair with the name of the stage and the data
      public: const std::pair<std::string, std::shared_ptr<sdf::usd::USDStage>>
          FindStage(const std::string &_name) const;

      public: friend std::ostream& operator<<(
        std::ostream& os, const USDData& data)
      {
        os << "References:" << "\n";
        for (auto &ref : data.AllReferences())
        {
          os << "\t" << ref.first << "\n";
          os << "\t\t" << ref.second->UpAxis() << "\n";
          os << "\t\t" << ref.second->MetersPerUnit() << "\n";
          for (auto &path : ref.second->USDPaths())
          {
            os << "\t\tPath " << path << '\n';
          }
        }
        os << "Models:" << "\n";
        auto models = data.Models();
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
