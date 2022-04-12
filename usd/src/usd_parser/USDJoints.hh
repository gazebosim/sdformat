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

#ifndef USD_PARSER_JOINTS_HH
#define USD_PARSER_JOINTS_HH

#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/usd/usdGeom/gprim.h>
#pragma pop_macro ("__DEPRECATED")

#include "sdf/Joint.hh"
#include "sdf/usd/usd_parser/USDData.hh"
#include "sdf/config.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  namespace usd
  {
    /// \brief Parse a USD joint to its SDF representation
    /// \param[in] _prim The USD prim that holds the USD joint
    /// \param[in] _path The path to _prim
    /// \param[in] _usdData Object that holds data about the USD stage
    sdf::Joint ParseJoints(
      const pxr::UsdPrim &_prim,
      const std::string &_path,
      const USDData &_usdData);
  }
  }
}

#endif
