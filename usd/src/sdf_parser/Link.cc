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

#include "sdf/usd/sdf_parser/Link.hh"

#include <string>

#include <ignition/math/Pose3.hh>

// TODO(ahcorde) this is to remove deprecated "warnings" in usd, these warnings
// are reported using #pragma message so normal diagnostic flags cannot remove
// them. This workaround requires this block to be used whenever usd is
// included.
#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/base/gf/quatf.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/xform.h>
#include <pxr/usd/usdPhysics/massAPI.h>
#include <pxr/usd/usdPhysics/rigidBodyAPI.h>
#pragma pop_macro ("__DEPRECATED")

#include "sdf/Link.hh"
#include "sdf/usd/sdf_parser/Light.hh"
#include "../UsdUtils.hh"

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
//
namespace usd
{
  UsdErrors ParseSdfLink(const sdf::Link &_link, pxr::UsdStageRefPtr &_stage,
      const std::string &_path, bool _rigidBody)
  {
    const pxr::SdfPath sdfLinkPath(_path);
    UsdErrors errors;

    auto usdLinkXform = pxr::UsdGeomXform::Define(_stage, sdfLinkPath);

    ignition::math::Pose3d pose;
    auto poseErrors = sdf::usd::PoseWrtParent(_link, pose);
    if (!poseErrors.empty())
    {
      errors.insert(errors.end(), poseErrors.begin(), poseErrors.end());
      return errors;
    }

    poseErrors = usd::SetPose(pose, _stage, sdfLinkPath);
    if (!poseErrors.empty())
    {
      errors.insert(errors.end(), poseErrors.begin(), poseErrors.end());
      errors.push_back(UsdError(UsdErrorCode::SDF_TO_USD_PARSING_ERROR,
            "Unable to set the pose of the link prim corresponding to the "
            "SDF link named [" + _link.Name() + "]"));
      return errors;
    }

    if (_rigidBody)
    {
      auto linkPrim = _stage->GetPrimAtPath(sdfLinkPath);
      if (!linkPrim)
      {
        errors.push_back(UsdError(sdf::usd::UsdErrorCode::INVALID_PRIM_PATH,
              "Internal error: unable to get prim at path ["
              + _path + "], but a link prim should exist at this path"));
        return errors;
      }

      if (!pxr::UsdPhysicsRigidBodyAPI::Apply(linkPrim))
      {
        errors.push_back(UsdError(sdf::usd::UsdErrorCode::FAILED_PRIM_API_APPLY,
              "Internal error: unable to mark link at path [" + _path
              + "] as a rigid body, so mass properties won't be attached"));
        return errors;
      }

      auto massAPI =
        pxr::UsdPhysicsMassAPI::Apply(linkPrim);
      if (!massAPI)
      {
        errors.push_back(UsdError(sdf::usd::UsdErrorCode::FAILED_PRIM_API_APPLY,
              "Unable to attach mass properties to link ["
              + _link.Name() + "]"));
        return errors;
      }
      massAPI.CreateMassAttr().Set(
          static_cast<float>(_link.Inertial().MassMatrix().Mass()));

      const auto diagonalInertial =
        _link.Inertial().MassMatrix().PrincipalMoments();
      massAPI.CreateDiagonalInertiaAttr().Set(pxr::GfVec3f(
        diagonalInertial[0], diagonalInertial[1], diagonalInertial[2]));

      const auto principalAxes =
        _link.Inertial().MassMatrix().PrincipalAxesOffset();
      massAPI.CreatePrincipalAxesAttr().Set(pxr::GfQuatf(
            principalAxes.W(),
            principalAxes.X(),
            principalAxes.Y(),
            principalAxes.Z()));

      const auto centerOfMass = _link.Inertial().Pose();
      massAPI.CreateCenterOfMassAttr().Set(pxr::GfVec3f(
        centerOfMass.Pos().X(),
        centerOfMass.Pos().Y(),
        centerOfMass.Pos().Z()));
    }

    // links can have lights attached to them
    for (uint64_t i = 0; i < _link.LightCount(); ++i)
    {
      const auto light = *(_link.LightByIndex(i));
      auto lightPath = std::string(_path + "/" + light.Name());
      UsdErrors lightErrors = ParseSdfLight(light, _stage, lightPath);
      if (!lightErrors.empty())
      {
        errors.push_back(
            UsdError(sdf::usd::UsdErrorCode::SDF_TO_USD_PARSING_ERROR,
              "Error parsing light [" + light.Name() + "]"));
        errors.insert(errors.end(), lightErrors.begin(), lightErrors.end());
      }
    }

    return errors;
  }
}
}
}
