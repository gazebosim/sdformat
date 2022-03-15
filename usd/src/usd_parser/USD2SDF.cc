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

#include "USD2SDF.hh"

#include "sdf/Console.hh"
#include "sdf/Types.hh"
#include "USDWorld.hh"

#include "sdf/Error.hh"
#include "sdf/Root.hh"
#include "sdf/World.hh"

#include "sdf/usd/UsdError.hh"

namespace sdf {
inline namespace SDF_VERSION_NAMESPACE {
namespace usd {
////////////////////////////////////////////////
UsdErrors USD2SDF::Read(const std::string &_fileName,
  sdf::Root &_root)
{
  UsdErrors errors;

  sdf::World sdfWorld;

  errors = parseUSDWorld(_fileName, sdfWorld);
  if (!errors.empty())
  {
    errors.emplace_back(UsdError(
      UsdErrorCode::SDF_TO_USD_PARSING_ERROR,
      "Error parsing usd file [" + _fileName + "]"));
    return errors;
  }

  const auto addWorldErrors = _root.AddWorld(sdfWorld);
  if (!addWorldErrors.empty())
  {
    for (const auto & error: addWorldErrors)
    {
      errors.emplace_back(error);
    }
    errors.emplace_back(UsdError(sdf::Error(
      sdf::ErrorCode::ELEMENT_INVALID,
      "Error adding the world [" + sdfWorld.Name() + "] to the SDF DOM")));
  }

  return errors;
}
}
}
}
