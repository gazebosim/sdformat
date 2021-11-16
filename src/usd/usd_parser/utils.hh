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

#ifndef USD_MODEL_UTILS_HH
#define USD_MODEL_UTILS_HH

#include <tuple>

#include <pxr/base/gf/vec3d.h>
#include <pxr/base/gf/matrix4d.h>
#include <pxr/base/gf/rotation.h>

#include <pxr/usd/usdGeom/gprim.h>

#include <ignition/math/Quaternion.hh>

#include "sdf/Material.hh"

namespace usd{

  std::string directoryFromUSDPath(std::string &_primPath);
  void removeSubStr(std::string &_str, const std::string &_substr);

  sdf::Material ParseMaterial(const pxr::UsdPrim &_prim);

  std::tuple<pxr::GfVec3f, pxr::GfVec3f, pxr::GfQuatf, bool, bool, bool, bool> ParseTransform(
    const pxr::UsdPrim &_prim);
}

#endif
