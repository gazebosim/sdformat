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

#ifndef SDF_USD_WORLD_HH_
#define SDF_USD_WORLD_HH_

#include <string>

#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/usd/usd/stage.h>
#pragma pop_macro ("__DEPRECATED")

#include "sdf/World.hh"

namespace usd
{
  /// \brief Parse an SDF world into a USD stage.
  /// \param[in] _world The SDF world to parse.
  /// \param[in] _stage The stage that should contain the USD representation
  /// of _world.
  /// \param[in] _path The USD path of the parsed world in _stage, which must be
  /// a valid USD path.
  /// \return True if _world was succesfully parsed into _stage with a path of
  /// _path. False otherwise.
  sdf::Errors SDFORMAT_VISIBLE ParseSdfWorld(const sdf::World &_world,
      pxr::UsdStageRefPtr &_stage, const std::string &_path);
}

#endif
