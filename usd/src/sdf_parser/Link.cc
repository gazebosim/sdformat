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
// TODO(ahcorde):this is to remove deprecated "warnings" in usd, these warnings
// are reported using #pragma message so normal diagnostic flags cannot remove
// them. This workaround requires this block to be used whenever usd is
// included.
#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/xform.h>
#include <pxr/usd/usdPhysics/massAPI.h>
#include <pxr/usd/usdPhysics/rigidBodyAPI.h>
#pragma pop_macro ("__DEPRECATED")

#include "sdf/Link.hh"
#include "sdf/usd/sdf_parser/Light.hh"
#include "sdf/usd/sdf_parser/Utils.hh"
#include "sdf/usd/sdf_parser/Visual.hh"

namespace sdf
{
// Inline bracke to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
//
namespace usd
{
  sdf::Errors ParseSdfLink(const sdf::Link &_link, pxr::UsdStageRefPtr &_stage,
      const std::string &_path, const bool _rigidBody)
  {
    const pxr::SdfPath sdfLinkPath(_path);
    sdf::Errors errors;

    auto usdLinkXform = pxr::UsdGeomXform::Define(_stage, sdfLinkPath);
    usd::SetPose(usd::PoseWrtParent(_link), _stage, sdfLinkPath);

    if (_rigidBody)
    {
      auto linkPrim = _stage->GetPrimAtPath(sdfLinkPath);
      if (!linkPrim)
      {
        errors.push_back(sdf::Error(sdf::ErrorCode::ATTRIBUTE_INCORRECT_TYPE,
              "Internal error: unable to get prim at path ["
              + _path + "], but a link prim should exist at this path"));
        return errors;
      }

      if (!pxr::UsdPhysicsRigidBodyAPI::Apply(linkPrim))
      {
        errors.push_back(sdf::Error(sdf::ErrorCode::ATTRIBUTE_INCORRECT_TYPE,
              "Internal error: unable to mark link at path [" + _path
              + "] as a rigid body, so mass properties won't be attached"));
        return errors;
      }

      auto massAPI =
        pxr::UsdPhysicsMassAPI::Apply(linkPrim);
      if (!massAPI)
      {
        errors.push_back(sdf::Error(sdf::ErrorCode::ATTRIBUTE_INCORRECT_TYPE,
              "Unable to attach mass properties to link ["
              + _link.Name() + "]"));
        return errors;
      }
      massAPI.CreateMassAttr().Set(
          static_cast<float>(_link.Inertial().MassMatrix().Mass()));

      const auto diagonalInertial =
        _link.Inertial().MassMatrix().DiagonalMoments();
      massAPI.CreateDiagonalInertiaAttr().Set(pxr::GfVec3f(
        diagonalInertial[0], diagonalInertial[1], diagonalInertial[2]));

      const auto centerOfMass = _link.Inertial().Pose();
      massAPI.CreateCenterOfMassAttr().Set(pxr::GfVec3f(
        centerOfMass.Pos().X(),
        centerOfMass.Pos().Y(),
        centerOfMass.Pos().Z()));
    }

    // TODO(adlarkin) finish parsing link. It will look something like this
    // (this does not cover all elements of a link that need to be parsed):
    //  * ParseSdfVisual
    //  * ParseSdfCollision

    // parse all of the link's visuals and convert them to USD
    for (uint64_t i = 0; i < _link.VisualCount(); ++i)
    {
      const auto visual = *(_link.VisualByIndex(i));
      const auto visualPath = std::string(_path + "/" + visual.Name());
      auto errorsLink = ParseSdfVisual(visual, _stage, visualPath);
      if (errorsLink.size() > 0)
      {
        errors.insert(errors.end(), errorsLink.begin(), errorsLink.end() );
        errors.push_back(sdf::Error(sdf::ErrorCode::ATTRIBUTE_INCORRECT_TYPE,
          "Error parsing visual [" + visual.Name() + "]"));
        return errors;
      }
    }

    // links can have lights attached to them
    for (uint64_t i = 0; i < _link.LightCount(); ++i)
    {
      const auto light = *(_link.LightByIndex(i));
      auto lightPath = std::string(_path + "/" + light.Name());
      sdf::Errors lightErrors = ParseSdfLight(light, _stage, lightPath);
      if (errors.size() > 0)
      {
        errors.push_back(sdf::Error(sdf::ErrorCode::ATTRIBUTE_INCORRECT_TYPE,
              "Error parsing light [" + light.Name() + "]"));
        errors.insert(errors.end(), lightErrors.begin(), lightErrors.end());
      }
    }

    return errors;
  }
}
}
}