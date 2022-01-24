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

#ifndef SDF_USD_USD_PARSER_UTILS_HH_
#define SDF_USD_USD_PARSER_UTILS_HH_

#include <string>

// TODO(ahcorde):this is to remove deprecated "warnings" in usd, these warnings
// are reported using #pragma message so normal diagnostic flags cannot remove
// them. This workaround requires this block to be used whenever usd is
// included.
#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/usd/usdGeom/gprim.h>
#pragma pop_macro ("__DEPRECATED")

#include "sdf/Material.hh"

namespace sdf
{
  // Inline bracke to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  namespace usd
  {
    /// Remove all substring appearances in the input string
    /// \param[in] _str Original string where the substring will be removed
    /// \param[in] _substr Substring to use to find and remove in the original
    /// \return A new string without the substring
    std::string SDFORMAT_VISIBLE removeSubStr(const std::string &_str,
        const std::string &_substr);

    /// brief Parse the material in a usdprim
    /// If the prim is a Geom then get the color values, or
    /// if the prim is a shade Material then get the texture values
    /// \param[in] _prim USD prim where the material is extracted
    /// \return Material of the prim
    sdf::Material SDFORMAT_VISIBLE ParseMaterial(const pxr::UsdPrim &_prim);
}
}
}
#endif
