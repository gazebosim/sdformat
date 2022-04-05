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

#include "Link.hh"

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
#include "../UsdUtils.hh"
#include "Collision.hh"
#include "Light.hh"
#include "Sensor.hh"
#include "Visual.hh"

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

      if (!pxr::UsdPhysicsRigidBodyAPI::Apply(linkPrim.GetParent()))
      {
        errors.push_back(UsdError(sdf::usd::UsdErrorCode::FAILED_PRIM_API_APPLY,
              "Internal error: unable to mark model at path [" +
              linkPrim.GetPath().GetString() + "] as a rigid body, "
              "so mass properties won't be attached"));
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

    // parse all of the link's visuals and convert them to USD
    for (uint64_t i = 0; i < _link.VisualCount(); ++i)
    {
      const auto visual = *(_link.VisualByIndex(i));
      const auto visualPath = std::string(_path + "/" + visual.Name());
      auto errorsLink = ParseSdfVisual(visual, _stage, visualPath);
      if (!errorsLink.empty())
      {
        errors.insert(errors.end(), errorsLink.begin(), errorsLink.end());
        errors.push_back(UsdError(
          sdf::usd::UsdErrorCode::SDF_TO_USD_PARSING_ERROR,
          "Error parsing visual [" + visual.Name() + "]"));
        return errors;
      }
    }

    // parse all of the link's collisions and convert them to USD
    for (uint64_t i = 0; i < _link.CollisionCount(); ++i)
    {
      const auto collision = *(_link.CollisionByIndex(i));
      const auto collisionPath = std::string(_path + "/" + collision.Name());
      auto errorsCollision = ParseSdfCollision(collision, _stage,
          collisionPath);
      if (!errorsCollision.empty())
      {
        errors.insert(errors.end(), errorsCollision.begin(),
            errorsCollision.end());
        errors.push_back(UsdError(
          sdf::usd::UsdErrorCode::SDF_TO_USD_PARSING_ERROR,
          "Error parsing collision [" + collision.Name()
          + "] attached to link [" + _link.Name() + "]"));
        return errors;
      }
    }

    // convert the link's sensors
    for (uint64_t i = 0; i < _link.SensorCount(); ++i)
    {
      const auto sensor = *(_link.SensorByIndex(i));
      const auto sensorPath = std::string(_path + "/" + sensor.Name());
      UsdErrors errorsSensor = ParseSdfSensor(sensor, _stage, sensorPath);
      if (!errorsSensor.empty())
      {
        errors.push_back(
            UsdError(sdf::usd::UsdErrorCode::SDF_TO_USD_PARSING_ERROR,
              "Error parsing sensor [" + sensor.Name() + "]"));
        errors.insert(errors.end(), errorsSensor.begin(), errorsSensor.end());
        return errors;
      }
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
