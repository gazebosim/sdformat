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

#ifndef SDF_USD_USD_PARSER_USDMATERIAL_HH_
#define SDF_USD_USD_PARSER_USDMATERIAL_HH_

// TODO(ahcorde) this is to remove deprecated "warnings" in usd, these warnings
// are reported using #pragma message so normal diagnostic flags cannot remove
// them. This workaround requires this block to be used whenever usd is
// included.
#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/usd/usdGeom/gprim.h>
#pragma pop_macro ("__DEPRECATED")

#include "sdf/Material.hh"
#include "sdf/config.hh"
#include "sdf/usd/UsdError.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  namespace usd
  {
    /// brief Parse the material in a USD prim to a sdf::Material
    /// If the prim is a pxr::UsdGeomGprim, get the color values. Otherwise,
    /// if the prim is a pxr::UsdShadeMaterial, get the texture values
    /// \param[in] _prim USD prim where the material is extracted
    /// \param[out] _material The sdf::Material representation of _prim's
    /// material
    /// \return UsdErrors, which is a vector of UsdError objects. Each UsdError
    /// includes an error code and message. An empty vector indicates no error.
    UsdErrors ParseMaterial(const pxr::UsdPrim &_prim,
        sdf::Material &_material);
}
}
}
#endif
