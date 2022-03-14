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

#ifndef USD_PARSER_PHYSYCS_HH
#define USD_PARSER_PHYSYCS_HH

#include "usd_model/WorldInterface.hh"

#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/usd/usdPhysics/scene.h>
#pragma pop_macro ("__DEPRECATED")

#include "sdf/config.hh"
#include "sdf/usd/Export.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  namespace usd
  {
    /// \brief Parse the physics attributes of a USD file
    /// \param[in] _scene USD physics scene to extract attributes from
    /// \param[out] _world World interface where the data is placed
    /// \param[in] _metersPerUnit meters per unit in the USD
    void IGNITION_SDFORMAT_USD_VISIBLE ParseUSDPhysicsScene(
      const pxr::UsdPhysicsScene &_scene,
      WorldInterface &_world,
      double _metersPerUnit);
  }
  }
}

#endif
