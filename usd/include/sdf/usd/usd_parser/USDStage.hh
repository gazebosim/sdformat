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

#ifndef SDF_USD_USD_PARSER_USDSTAGE_HH_
#define SDF_USD_USD_PARSER_USDSTAGE_HH_

#include <set>
#include <string>

#include <ignition/utils/ImplPtr.hh>

#include "sdf/Types.hh"
#include "sdf/sdf_config.h"
#include "sdf/system_util.hh"
#include "sdf/usd/Export.hh"
#include "sdf/usd/UsdError.hh"

namespace sdf
{
  // Inline bracke to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  namespace usd
  {
    /// \brief this class will keep the data of a stage
    /// - UpAxis
    /// - MetersPerUnit
    /// - All USD paths
    class IGNITION_SDFORMAT_USD_VISIBLE USDStage
    {
      /// \brief Default constructor
      /// \param[in] _refFileName File name of the stage in the disk
      public: USDStage(const std::string &_refFileName);

      /// \brief Initialize the data structure
      /// \return Errors, which is a vector of Error objects. Each Error includes
      /// an error code and message. An empty vector indicates no error.
      public: UsdErrors Init();

      /// \brief Get stage up axis
      public: const std::string &GetUpAxis() const;

      /// \brief Get stage meter per unit
      public: double GetMetersPerUnit() const;

      /// \brief Get USD paths available in the stage
      public: const std::set<std::string> &GetUSDPaths() const;

      /// \brief Private data pointer.
      IGN_UTILS_IMPL_PTR(dataPtr)
    };
  }
  }
}
#endif  // SDF_USD_PARSER_USDSTAGE_HH
