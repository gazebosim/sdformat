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

#ifndef SDF_PARSER_MATERIAL_HH_
#define SDF_PARSER_MATERIAL_HH_

#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdShade/material.h>

#include "sdf/Material.hh"
#include "sdf/sdf_config.h"

namespace usd
{
  /// \brief Parse an SDF material into a USD stage.
  /// \param[in] _material The SDF material to parse.
  /// \param[in] _stage The stage that should contain the USD representation
  /// of _material.
  /// \return The parsed usd material (invalid material if parsing failed)
  pxr::UsdShadeMaterial SDFORMAT_VISIBLE
  ParseSdfMaterial(const sdf::Material *_material,
      pxr::UsdStageRefPtr &_stage);
}

#endif
