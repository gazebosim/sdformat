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

#ifndef USD_USD_PARSER_LIGHTS_HH
#define USD_USD_PARSER_LIGHTS_HH

#include <memory>
#include <string>

#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/usd/usd/prim.h>
#pragma pop_macro ("__DEPRECATED")

#include "sdf/Light.hh"

#include "sdf/config.hh"
#include "sdf/usd/Export.hh"

#include "sdf/usd/usd_parser/USDData.hh"

namespace sdf
{
  // Inline bracke to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  namespace usd
  {
    /// \brief Parse lights
    /// Supported lights:
    /// - UsdLuxDistantLight
    /// - UsdLuxDiskLight
    /// \param[in] _prim Prim to extract the light data
    /// \param[in] _usdData Object to get transform data
    /// \param[in] _linkName Name of the link to find the transform
    /// \return Shared point with the sdf Light object or null if the light
    /// is not supported.
    std::shared_ptr<sdf::Light> IGNITION_SDFORMAT_USD_VISIBLE ParseUSDLights(
      const pxr::UsdPrim &_prim,
      USDData &_usdData,
      const std::string &_linkName);
  }
}
}

#endif
