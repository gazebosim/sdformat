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

#include "sdf/usd/sdf_parser/Joint.hh"

#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

// TODO(ahcorde) this is to remove deprecated "warnings" in usd, these warnings
// are reported using #pragma message so normal diagnostic flags cannot remove
// them. This workaround requires this block to be used whenever usd is
// included.
#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/base/gf/quatf.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/tf/token.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/relationship.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdPhysics/driveAPI.h>
#include <pxr/usd/usdPhysics/fixedJoint.h>
#include <pxr/usd/usdPhysics/joint.h>
#include <pxr/usd/usdPhysics/prismaticJoint.h>
#include <pxr/usd/usdPhysics/revoluteJoint.h>
#include <pxr/usd/usdPhysics/sphericalJoint.h>
#pragma pop_macro ("__DEPRECATED")

#include "sdf/Error.hh"
#include "sdf/Joint.hh"
#include "sdf/JointAxis.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"
#include "../UsdUtils.hh"

namespace sdf
{
// Inline bracke to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
//
namespace usd
{
  /// \brief Helper function for setting a USD joint's pose relative to the
  /// joint's parent and child.
  /// \param[in] _jointPrim The USD joint prim
  /// \param[in] _joint The SDF representation of _jointPrim
  /// \param[in] _parentModel The SDF model that is the parent of _joint
  /// \return UsdErrors, which is a vector of UsdError objects. Each UsdError
  /// includes an error code and message. An empty vector indicates no error
  /// occurred when setting _joint's pose relative to _joint's parent and child
  UsdErrors SetUSDJointPose(pxr::UsdPhysicsJoint &_jointPrim,
      const sdf::Joint &_joint, const sdf::Model &_parentModel)
  {
    UsdErrors errors;

    ignition::math::Pose3d parentToJoint;
    if (_joint.ParentLinkName() == "world")
    {
      ignition::math::Pose3d modelToJoint;
      auto poseErrors = usd::PoseWrtParent(_joint, modelToJoint);
      if (!poseErrors.empty())
        return poseErrors;

      // it is assumed the _parentModel's parent is the world
      ignition::math::Pose3d worldToModel;
      poseErrors = usd::PoseWrtParent(_parentModel, worldToModel);
      if (!poseErrors.empty())
        return poseErrors;

      parentToJoint = worldToModel * modelToJoint;
    }
    else
    {
      auto poseResolutionErrors =
        _joint.SemanticPose().Resolve(parentToJoint, _joint.ParentLinkName());
      if (!poseResolutionErrors.empty())
      {
        errors.push_back(UsdError(
              sdf::Error(sdf::ErrorCode::POSE_RELATIVE_TO_INVALID,
              "Unable to get the pose of joint [" + _joint.Name() +
              "] w.r.t. its parent link [" + _joint.ParentLinkName() + "].")));
        for (const auto &e : poseResolutionErrors)
          errors.push_back(UsdError(e));
        return errors;
      }
    }

    ignition::math::Pose3d childToJoint;
    auto poseResolutionErrors = _joint.SemanticPose().Resolve(childToJoint,
        _joint.ChildLinkName());
    if (!poseResolutionErrors.empty())
    {
      errors.push_back(UsdError(
          sdf::Error(sdf::ErrorCode::POSE_RELATIVE_TO_INVALID,
            "Unable to get the pose of joint [" + _joint.Name() +
            "] w.r.t. its child [" + _joint.ChildLinkName() + "].")));
      for (const auto &e : poseResolutionErrors)
        errors.push_back(UsdError(e));
      return errors;
    }

    auto parentRotation = pxr::GfQuatf(
          parentToJoint.Rot().W(),
          parentToJoint.Rot().X(),
          parentToJoint.Rot().Y(),
          parentToJoint.Rot().Z());

    auto childRotation = pxr::GfQuatf(
          childToJoint.Rot().W(),
          childToJoint.Rot().X(),
          childToJoint.Rot().Y(),
          childToJoint.Rot().Z());

    const auto axis = _joint.Axis();
    if (axis && (axis->Xyz() == ignition::math::Vector3d::UnitY))
    {
      if (auto jointRevolute = pxr::UsdPhysicsRevoluteJoint(_jointPrim))
      {
        const ignition::math::Quaterniond fixRotation(0, 0, IGN_DTOR(90));
        ignition::math::Quaterniond parentRotationTmp = parentToJoint.Rot();
        ignition::math::Quaterniond childRotationTmp = childToJoint.Rot();

        if (parentRotationTmp == ignition::math::Quaterniond::Identity)
        {
          parentRotationTmp = fixRotation * parentRotationTmp;
        }
        else
        {
          parentRotationTmp = ignition::math::Quaterniond(IGN_DTOR(-90),
              IGN_PI, IGN_PI) * parentRotationTmp;
        }

        childRotationTmp = fixRotation * childRotationTmp;

        parentRotation = pxr::GfQuatf(
            parentRotationTmp.W(),
            parentRotationTmp.X(),
            parentRotationTmp.Y(),
            parentRotationTmp.Z());
        childRotation = pxr::GfQuatf(
            childRotationTmp.W(),
            childRotationTmp.X(),
            childRotationTmp.Y(),
            childRotationTmp.Z());
        jointRevolute.CreateAxisAttr().Set(pxr::TfToken("X"));
      }
    }

    _jointPrim.CreateLocalPos0Attr().Set(pxr::GfVec3f(
          parentToJoint.Pos().X(),
          parentToJoint.Pos().Y(),
          parentToJoint.Pos().Z()));
    _jointPrim.CreateLocalRot0Attr().Set(parentRotation);

    _jointPrim.CreateLocalPos1Attr().Set(pxr::GfVec3f(
          childToJoint.Pos().X(),
          childToJoint.Pos().Y(),
          childToJoint.Pos().Z()));
    _jointPrim.CreateLocalRot1Attr().Set(childRotation);

    return errors;
  }

  /// \brief Helper function to parse a SDF revolute joint to its USD
  /// representation.
  /// \param[in] _joint The SDF joint to parse
  /// \param[in] _stage The stage that _joint should belong to. This must be a
  /// valid, initialized stage.
  /// \param[in] _path The path in _stage where the USD representation of _joint
  /// will be defined.
  /// \return UsdErrors, which is a vector of UsdError objects. Each UsdError
  /// includes an error code and message. An empty vector indicates no error
  /// occurred when converting _joint to its USD representation.
  UsdErrors ParseSdfRevoluteJoint(const sdf::Joint &_joint,
      pxr::UsdStageRefPtr &_stage, const std::string &_path)
  {
    UsdErrors errors;

    auto usdJoint =
      pxr::UsdPhysicsRevoluteJoint::Define(_stage, pxr::SdfPath(_path));

    const auto axis = _joint.Axis();

    if (axis->Xyz() == ignition::math::Vector3d::UnitX ||
        axis->Xyz() == -ignition::math::Vector3d::UnitX)
    {
      usdJoint.CreateAxisAttr().Set(pxr::TfToken("X"));
    }
    else if (axis->Xyz() == ignition::math::Vector3d::UnitY ||
        axis->Xyz() == -ignition::math::Vector3d::UnitY)
    {
      usdJoint.CreateAxisAttr().Set(pxr::TfToken("Y"));
    }
    else if (axis->Xyz() == ignition::math::Vector3d::UnitZ ||
        axis->Xyz() == -ignition::math::Vector3d::UnitZ)
    {
      usdJoint.CreateAxisAttr().Set(pxr::TfToken("Z"));
    }
    else
    {
      errors.push_back(UsdError(sdf::Error(sdf::ErrorCode::ELEMENT_INVALID,
            "Revolute joint [" + _joint.Name() + "] has an invalid axis.")));
      return errors;
    }

    // Revolute joint limits in SDF are in radians, but USD expects degrees
    // of C++ type float
    auto sdfLimitDegrees = static_cast<float>(IGN_RTOD(axis->Lower()));
    usdJoint.CreateLowerLimitAttr().Set(sdfLimitDegrees);
    sdfLimitDegrees = static_cast<float>(IGN_RTOD(axis->Upper()));
    usdJoint.CreateUpperLimitAttr().Set(sdfLimitDegrees);

    pxr::UsdPrim usdJointPrim = _stage->GetPrimAtPath(pxr::SdfPath(_path));

    auto drive =
      pxr::UsdPhysicsDriveAPI::Apply(usdJointPrim, pxr::TfToken("angular"));
    if (!drive)
    {
      errors.push_back(UsdError(sdf::usd::UsdErrorCode::FAILED_PRIM_API_APPLY,
            "Internal error: unable to mark link at path [" + _path +
            "] as a UsdPhysicsDriveAPI."));
      return errors;
    }

    // TODO(ahcorde) Review damping and stiffness values
    // I added these values as a proof of concept.
    double damping = axis->Damping();
    if (ignition::math::equal(damping, 0.0))
    {
      damping = 35;
    }
    drive.CreateDampingAttr().Set(static_cast<float>(damping));
    double stiffness = axis->Stiffness();
    if (ignition::math::equal(stiffness, 1e8))
    {
      stiffness = 350;
    }
    drive.CreateStiffnessAttr().Set(static_cast<float>(stiffness));
    drive.CreateMaxForceAttr().Set(static_cast<float>(axis->Effort()));

    return errors;
  }

  /// \brief Helper function to parse a SDF prismatic joint to its USD
  /// representation.
  /// \param[in] _joint The SDF joint to parse
  /// \param[in] _stage The stage that _joint should belong to. This must be a
  /// valid, initialized stage.
  /// \param[in] _path The path in _stage where the USD representation of _joint
  /// will be defined.
  /// \return UsdErrors, which is a vector of UsdError objects. Each UsdError
  /// includes an error code and message. An empty vector indicates no error
  /// occurred when converting _joint to its USD representation.
  UsdErrors ParseSdfPrismaticJoint(const sdf::Joint &_joint,
      pxr::UsdStageRefPtr &_stage, const std::string &_path)
  {
    UsdErrors errors;

    auto usdJoint =
      pxr::UsdPhysicsPrismaticJoint::Define(_stage, pxr::SdfPath(_path));

    const auto axis = _joint.Axis();

    if (axis->Xyz() == ignition::math::Vector3d::UnitX ||
        axis->Xyz() == -ignition::math::Vector3d::UnitX)
    {
      usdJoint.CreateAxisAttr().Set(pxr::TfToken("X"));
    }
    else if (axis->Xyz() == ignition::math::Vector3d::UnitY ||
        axis->Xyz() == -ignition::math::Vector3d::UnitY)
    {
      usdJoint.CreateAxisAttr().Set(pxr::TfToken("Y"));
    }
    else if (axis->Xyz() == ignition::math::Vector3d::UnitZ ||
        axis->Xyz() == -ignition::math::Vector3d::UnitZ)
    {
      usdJoint.CreateAxisAttr().Set(pxr::TfToken("Z"));
    }
    else
    {
      errors.push_back(UsdError(sdf::Error(sdf::ErrorCode::ELEMENT_INVALID,
            "Prismatic joint [" + _joint.Name() + "] has an invalid axis.")));
      return errors;
    }

    usdJoint.CreateLowerLimitAttr().Set(
        static_cast<float>(axis->Lower()));
    usdJoint.CreateUpperLimitAttr().Set(
        static_cast<float>(axis->Upper()));

    return errors;
  }

  UsdErrors ParseSdfJoint(const sdf::Joint &_joint,
      pxr::UsdStageRefPtr &_stage, const std::string &_path,
      const sdf::Model &_parentModel,
      const std::unordered_map<std::string, pxr::SdfPath> &_linkToUsdPath,
      const pxr::SdfPath &_worldPath)
  {
    UsdErrors errors;

    // the joint's parent may be "world". If this is the case, the joint's
    // parent should be set to the world prim, not a link
    auto parentLinkPath = _worldPath;
    if (_joint.ParentLinkName() != "world")
    {
      const auto it = _linkToUsdPath.find(_joint.ParentLinkName());
      if (it == _linkToUsdPath.end())
      {
        errors.push_back(UsdError(sdf::usd::UsdErrorCode::INVALID_PRIM_PATH,
              "Unable to find a USD path for link [" + _joint.ParentLinkName() +
              "], which is the parent link of joint [" + _joint.Name() + "]."));
        return errors;
      }
      parentLinkPath = it->second;
    }

    const auto it = _linkToUsdPath.find(_joint.ChildLinkName());
    if (it == _linkToUsdPath.end())
    {
      errors.push_back(UsdError(sdf::usd::UsdErrorCode::INVALID_PRIM_PATH,
            "Unable to find a USD path for link [" + _joint.ParentLinkName() +
            "], which is the child link of joint [" + _joint.Name() + "]."));
      return errors;
    }
    const auto childLinkPath = it->second;

    UsdErrors parsingErrors;
    switch (_joint.Type())
    {
      case sdf::JointType::REVOLUTE:
        parsingErrors = ParseSdfRevoluteJoint(_joint, _stage, _path);
        break;
      case sdf::JointType::BALL:
        // While USD allows for cone limits that can restrict motion in a given
        // range, SDF does not have limits for a ball joint. So, there's
        // nothing to do after creating a UsdPhysicsSphericalJoint, since this
        // joint by default has no limits (i.e., allows for circular motion)
        pxr::UsdPhysicsSphericalJoint::Define(_stage, pxr::SdfPath(_path));
        break;
      case sdf::JointType::FIXED:
        pxr::UsdPhysicsFixedJoint::Define(_stage, pxr::SdfPath(_path));
        break;
      case sdf::JointType::PRISMATIC:
        parsingErrors = ParseSdfPrismaticJoint(_joint, _stage, _path);
        break;
      case sdf::JointType::CONTINUOUS:
      case sdf::JointType::GEARBOX:
      case sdf::JointType::REVOLUTE2:
      case sdf::JointType::SCREW:
      case sdf::JointType::UNIVERSAL:
      case sdf::JointType::INVALID:
      default:
        parsingErrors.push_back(UsdError(
              sdf::Error(sdf::ErrorCode::ATTRIBUTE_INVALID,
              "Joint type is either invalid or not supported.")));
    }

    if (!parsingErrors.empty())
    {
      errors.insert(errors.end(), parsingErrors.begin(), parsingErrors.end());
      return errors;
    }

    auto jointPrim = pxr::UsdPhysicsJoint::Get(_stage, pxr::SdfPath(_path));
    if (!jointPrim)
    {
      errors.push_back(UsdError(sdf::usd::UsdErrorCode::INVALID_PRIM_PATH,
            "Internal error: unable to get prim at path [" + _path +
            "], but a joint prim should exist at this path."));
      return errors;
    }

    // define the joint's parent/child links
    jointPrim.CreateBody0Rel().AddTarget(parentLinkPath);
    jointPrim.CreateBody1Rel().AddTarget(childLinkPath);

    const auto poseErrors =
      SetUSDJointPose(jointPrim, _joint, _parentModel);
    if (!poseErrors.empty())
      errors.insert(errors.end(), poseErrors.begin(), poseErrors.end());

    return errors;
  }
}
}
}
