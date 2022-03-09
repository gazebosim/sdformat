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
#include <pxr/usd/usd/primRange.h>
#pragma pop_macro ("__DEPRECATED")

#include <sdf/sdf_config.h>

namespace sdf
{
  // Inline bracke to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  namespace usd
  {
    /// \brief It parses the physics attributes of the USD file
    /// \param[in] _prim Prim to extract the physics attributes
    /// \param[out] _world World interface where the data is placed
    /// \param[in] _metersPerUnit meter per unit in the USD
    void ParsePhysicsScene(
      const pxr::UsdPrim &_prim,
      std::shared_ptr<WorldInterface> &_world,
      double _metersPerUnit);
  }
  }
}

#endif
