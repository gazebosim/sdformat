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

#include "sdf/sdf_config.h"

namespace usd{

  std::string SDFORMAT_VISIBLE directoryFromUSDPath(std::string &_primPath);
  void SDFORMAT_VISIBLE removeSubStr(std::string &_str, const std::string &_substr);

  sdf::Material SDFORMAT_VISIBLE ParseMaterial(const pxr::UsdPrim &_prim);

  class SDFORMAT_VISIBLE Transforms
  {
  public:
    ignition::math::Vector3d scale{1, 1, 1};
    std::vector<ignition::math::Quaterniond> q;
    ignition::math::Vector3d translate{0, 0, 0};
    bool isRotationZYX = false;
    bool isRotation = false;
    bool isTranslate = false;
  };

  Transforms SDFORMAT_VISIBLE ParseTransform(const pxr::UsdPrim &_prim);

  void SDFORMAT_VISIBLE GetTransform(
    const pxr::UsdPrim &_prim,
    const double _metersPerUnit,
    ignition::math::Pose3d &_pose,
    ignition::math::Vector3d &_scale,
    const std::string &_name);

  void SDFORMAT_VISIBLE GetAllTransforms(
    const pxr::UsdPrim &_prim,
    const double _metersPerUnit,
    std::vector<ignition::math::Pose3d> &_tfs,
    ignition::math::Vector3d &_scale,
    const std::string &_name);

  // std::tuple<pxr::GfVec3f, pxr::GfVec3f, pxr::GfQuatf, bool, bool, bool, bool> ParseTransform(
  //   const pxr::UsdPrim &_prim);
}

#endif
