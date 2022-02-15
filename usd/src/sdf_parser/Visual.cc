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

#include "sdf/usd/sdf_parser/Visual.hh"

#include <iostream>
#include <string>

#include <ignition/math/Pose3.hh>

// TODO(adlarkin):this is to remove deprecated "warnings" in usd, these warnings
// are reported using #pragma message so normal diagnostic flags cannot remove
// them. This workaround requires this block to be used whenever usd is
// included.
#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/xform.h>
#include <pxr/usd/usdShade/material.h>
#include <pxr/usd/usdShade/materialBindingAPI.h>
#pragma pop_macro ("__DEPRECATED")

#include "sdf/Visual.hh"
#include "sdf/usd/sdf_parser/Geometry.hh"
#include "../UsdUtils.hh"

namespace sdf
{
// Inline bracke to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
//
namespace usd
{
  UsdErrors ParseSdfVisual(const sdf::Visual &_visual,
      pxr::UsdStageRefPtr &_stage, const std::string &_path)
  {
    UsdErrors errors;
    const pxr::SdfPath sdfVisualPath(_path);
    auto usdVisualXform = pxr::UsdGeomXform::Define(_stage, sdfVisualPath);
    if (!usdVisualXform)
    {
      errors.push_back(UsdError(sdf::usd::UsdErrorCode::FAILED_USD_DEFINITION,
        "Not able to define a Geom Xform at path [" + _path + "]"));
      return errors;
    }

    ignition::math::Pose3d pose;
    auto poseErrors = usd::PoseWrtParent(_visual, pose);
    if (!poseErrors.empty())
    {
      for (const auto &e : poseErrors)
        errors.push_back(e);
      return errors;
    }

    poseErrors = usd::SetPose(pose, _stage, sdfVisualPath);
    if (!poseErrors.empty())
    {
      for (const auto &e : poseErrors)
        errors.push_back(e);
      errors.push_back(UsdError(UsdErrorCode::SDF_TO_USD_PARSING_ERROR,
            "Unable to set the pose of the link prim corresponding to the "
            "SDF visual named [" + _visual.Name() + "]"));
      return errors;
    }

    const auto geometry = *(_visual.Geom());
    const auto geometryPath = std::string(_path + "/geometry");
    auto geomErrors = ParseSdfGeometry(geometry, _stage, geometryPath);
    if (!geomErrors.empty())
    {
      errors.insert(errors.end(), geomErrors.begin(), geomErrors.end() );
      errors.push_back(UsdError(
        sdf::usd::UsdErrorCode::SDF_TO_USD_PARSING_ERROR,
        "Error parsing geometry attached to visual [" + _visual.Name() + "]"));
      return errors;
    }

    return errors;
  }
}
}
}
