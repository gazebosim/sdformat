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

#ifndef SDF_USD_USD_PARSER_PARSER_HH
#define SDF_USD_USD_PARSER_PARSER_HH

#include <string>

#include "sdf/config.hh"
#include "sdf/usd/Export.hh"
#include "sdf/usd/UsdError.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  namespace usd
  {
    /// \brief Parse a USD file and convert it to a SDF file
    /// \param[in] _inputFilenameUsd Path of the USD file to parse
    /// \param[in] _outputFilenameSdf Path where the SDF file will be located
    /// \return UsdErrors, which is a vector of UsdError objects. Each UsdError
    /// includes an error code and message. An empty vector indicates no error
    /// occurred when parsing the USD file to its SDF representation.
    UsdErrors IGNITION_SDFORMAT_USD_VISIBLE parseUSDFile(
      const std::string &_inputFilenameUsd,
      const std::string &_outputFilenameSdf);
  }
  }
}
#endif
