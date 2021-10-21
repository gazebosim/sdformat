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

#include "physics.hh"

#include "pxr/usd/usdPhysics/scene.h"

#include "sdf/Console.hh"

namespace usd
{
  void ParsePhysicsScene(const pxr::UsdPrim &_prim)
  {
    sdferr << "UsdPhysicsScene" << "\n";
    auto variant_physics_scene = pxr::UsdPhysicsScene(_prim);
    pxr::GfVec3d gravity;
    float magnitude;
    variant_physics_scene.GetGravityDirectionAttr().Get(&gravity);
    sdferr << "gravity " << gravity << "\n";
    variant_physics_scene.GetGravityMagnitudeAttr().Get(&magnitude);
    sdferr << "magnitude " << magnitude << "\n";
  }
}
