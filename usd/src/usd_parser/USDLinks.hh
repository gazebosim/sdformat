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

#ifndef SDF_USD_USD_PARSER_USD_LINKS_HH
#define SDF_USD_USD_PARSER_USD_LINKS_HH

#include <optional>
#include <string>

#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/usd/usdGeom/gprim.h>
#pragma pop_macro ("__DEPRECATED")

#include "sdf/config.hh"
#include "sdf/usd/usd_parser/USDData.hh"
#include "sdf/usd/UsdError.hh"
#include "sdf/Link.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  namespace usd
  {
    /// \brief Parse a USD link into its SDF representation
    /// \param[in] _prim The USD prim that contains the link
    /// \param[in] _nameLink The name of the link
    /// \param[out] _link The SDF link to populate. If no value is held, a new
    /// link will be created. If the optional does hold a value, the existing
    /// link will have additional information added to it
    /// \param[in] _usdData USDData object that holds data about the USD stage
    /// \param[in] _scale scale of current Link, sdf::Link does not have a scale
    /// attribute we need to keeep the scale in a extenal variable
    /// \return UsdErrors, which is a list of UsdError objects. An empty list
    /// means there were no errors parsing the USD link
    UsdErrors ParseUSDLinks(
      const pxr::UsdPrim &_prim,
      const std::string &_nameLink,
      std::optional<sdf::Link> &_link,
      const USDData &_usdData,
      ignition::math::Vector3d &_scale);
  }
  }
}

#endif
