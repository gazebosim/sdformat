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

#include "sdf/usd/usd_parser/Parser.hh"
#include "USD2SDF.hh"

#include "sdf/Root.hh"

namespace sdf
{
inline namespace SDF_VERSION_NAMESPACE {
namespace usd
{
  UsdErrors parseUSDFile(
      const std::string &_inputFilenameUsd,
      const std::string &_outputFilenameSdf)
  {
    UsdErrors errors;
    USD2SDF usd2sdf;
    sdf::Root root;
    errors = usd2sdf.Read(_inputFilenameUsd, root);
    if (!errors.empty())
    {
      return errors;
    }

    std::ofstream out(_outputFilenameSdf.c_str(), std::ios::out);
    std::string string = root.ToElement()->ToString("");
    if (!out)
    {
      errors.emplace_back(UsdError(
        UsdErrorCode::SDF_TO_USD_PARSING_ERROR,
        "Unable to open file [" + _outputFilenameSdf + "] for writing"));
      return errors;
    }
    out << string;
    out.close();
    return errors;
  }
}
}
}
