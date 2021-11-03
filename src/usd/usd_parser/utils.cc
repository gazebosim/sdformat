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

#include <iostream>

#include "utils.hh"

namespace usd
{
  sdf::Material ParseMaterial(const pxr::UsdPrim &_prim)
  {
    sdf::Material material;
    auto variant_geom = pxr::UsdGeomGprim(_prim);

    pxr::VtArray<pxr::GfVec3f> color {{0, 0, 0}};

    variant_geom.GetDisplayColorAttr().Get(&color);

    pxr::VtFloatArray displayOpacity;
    _prim.GetAttribute(pxr::TfToken("primvars:displayOpacity")).Get(&displayOpacity);

    variant_geom.GetDisplayColorAttr().Get(&color);
    double alpha = 1.0;
    if (displayOpacity.size() > 0)
    {
      alpha = 1 - displayOpacity[0];
    }
    material.SetAmbient(
      ignition::math::Color(
        ignition::math::clamp(color[0][2] / 0.4, 0.0, 1.0),
        ignition::math::clamp(color[0][1] / 0.4, 0.0, 1.0),
        ignition::math::clamp(color[0][0] / 0.4, 0.0, 1.0),
        alpha));
    material.SetDiffuse(
      ignition::math::Color(
        ignition::math::clamp(color[0][2] / 0.8, 0.0, 1.0),
        ignition::math::clamp(color[0][1] / 0.8, 0.0, 1.0),
        ignition::math::clamp(color[0][0] / 0.8, 0.0, 1.0),
        alpha));

    return material;
  }

}
