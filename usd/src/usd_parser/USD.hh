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

#ifndef USD_PARSER_USD_HH
#define USD_PARSER_USD_HH

#include <string>

#include "sdf/sdf_config.h"
#include "sdf/usd/UsdError.hh"

#include "usd_model/WorldInterface.hh"

namespace sdf
{
  // Inline bracke to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  namespace usd
  {
    /// \brief it parses the USD file
    /// \param[in] _inputFilename Path where the USD is located
    /// \param[out] _world World interface where all USD data is placed
    /// \return UsdErrors, which is a vector of UsdError objects. Each UsdError
    /// includes an error code and message. An empty vector indicates no error
    /// occurred when parsing to its SDF representation.
    UsdErrors parseUSDWorld(
      const std::string &_inputFilename,
      std::shared_ptr<WorldInterface> &_world);
  }
  }
}
#endif
