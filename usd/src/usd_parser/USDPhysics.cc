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

#include "USDPhysics.hh"

#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/base/gf/vec3d.h>
#include <pxr/usd/usdPhysics/scene.h>
#pragma pop_macro ("__DEPRECATED")

namespace sdf
{
inline namespace SDF_VERSION_NAMESPACE {
namespace usd
{
  void ParseUSDPhysicsScene(
    const pxr::UsdPhysicsScene &_scene,
    WorldInterface &_world,
    double _metersPerUnit)
  {
    const auto gravityAttr = _scene.GetGravityDirectionAttr();
    pxr::GfVec3f gravity;
    gravityAttr.Get(&gravity);
    if (!ignition::math::equal(0.0f, gravity[0]) &&
        !ignition::math::equal(0.0f, gravity[1]) &&
        !ignition::math::equal(0.0f, gravity[2]))
    {
      _world.gravity[0] = gravity[0];
      _world.gravity[1] = gravity[1];
      _world.gravity[2] = gravity[2];
    }

    const auto magnitudeAttr = _scene.GetGravityMagnitudeAttr();
    float magnitude;
    magnitudeAttr.Get(&magnitude);
    if (!std::isnan(magnitude) && !std::isinf(magnitude))
    {
      _world.magnitude = magnitude * _metersPerUnit;
    }
  }
}
}
}
