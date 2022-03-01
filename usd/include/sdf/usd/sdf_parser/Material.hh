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

#ifndef SDF_USD_SDF_PARSER_MATERIALS_HH_
#define SDF_USD_SDF_PARSER_MATERIALS_HH_

// TODO(ahcorde) this is to remove deprecated "warnings" in usd, these warnings
// are reported using #pragma message so normal diagnostic flags cannot remove
// them. This workaround requires this block to be used whenever usd is
// included.
#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdShade/material.h>
#pragma pop_macro ("__DEPRECATED")

#include "sdf/Material.hh"
#include "sdf/usd/Export.hh"
#include "sdf/usd/UsdError.hh"
#include "sdf/sdf_config.h"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  namespace usd
  {
    /// \brief Parse an SDF material into a USD stage.
    /// \param[in] _materialSdf The SDF material to parse.
    /// \param[in] _stage The stage that should contain the USD representation
    /// of _material.
    /// \param[out] _materialPath Material usd path
    /// \return UsdErrors, which is a list of UsdError objects. This list is
    /// empty if no errors occurred when parsing _materialSdf to _materialUsd
    UsdErrors IGNITION_SDFORMAT_USD_VISIBLE
      ParseSdfMaterial(
        const sdf::Material *_materialSdf,
        pxr::UsdStageRefPtr &_stage,
        std::string &_materialPath);
  }
  }
}

#endif
