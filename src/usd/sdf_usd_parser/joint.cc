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

#include "sdf_usd_parser/joint.hh"

#include <iostream>
#include <string>
#include <unordered_map>

#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>
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

#include "sdf/Joint.hh"
#include "sdf/JointAxis.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"
#include "sdf_usd_parser/utils.hh"

namespace usd
{
  /// \brief Helper function for setting a USD joint's pose relative to the
  /// joint's parent and child.
  /// \param[in] _jointPrim The USD joint prim
  /// \param[in] _joint The SDF representation of _jointPrim
  /// \param[in] _parentModel The SDF model that is the parent of _joint
  /// \return True if _joint's pose was properly set relative to the joint's
  /// parent and child. False otherwise
  bool SetUSDJointPose(pxr::UsdPhysicsJoint &_jointPrim,
      const sdf::Joint &_joint, const sdf::Model &_parentModel)
  {
    ignition::math::Pose3d parentToJoint;
    if (_joint.ParentLinkName() == "world")
    {
      const auto modelToJoint = usd::PoseWrtParent(_joint);
      // it is assumed the _parentModel's parent is the world
      const auto worldToModel = usd::PoseWrtParent(_parentModel);
      parentToJoint = worldToModel * modelToJoint;
    }
    else
    {
      auto errors =
        _joint.SemanticPose().Resolve(parentToJoint, _joint.ParentLinkName());
      if (!errors.empty())
      {
        std::cerr << "Unable to get the pose of joint [" << _joint.Name()
                  << "] w.r.t. its parent [" << _joint.ParentLinkName() << "]\n"
                  << "The following errors occurred during pose computation:"
                  << "\n\t" << errors;
        return false;
      }
    }
    _jointPrim.CreateLocalPos0Attr().Set(pxr::GfVec3f(
          parentToJoint.Pos().X(),
          parentToJoint.Pos().Y(),
          parentToJoint.Pos().Z()));
    _jointPrim.CreateLocalRot0Attr().Set(pxr::GfQuatf(
          parentToJoint.Rot().W(),
          parentToJoint.Rot().X(),
          parentToJoint.Rot().Y(),
          parentToJoint.Rot().Z()));

    ignition::math::Pose3d childToJoint;
    auto errors = _joint.SemanticPose().Resolve(childToJoint,
        _joint.ChildLinkName());
    if (!errors.empty())
    {
      std::cerr << "Unable to get the pose of joint [" << _joint.Name()
                << "] w.r.t. its child [" << _joint.ChildLinkName() << "]\n"
                << "The following errors occurred during pose computation:"
                << "\n\t" << errors;
      return false;
    }
    _jointPrim.CreateLocalPos1Attr().Set(pxr::GfVec3f(
          childToJoint.Pos().X(),
          childToJoint.Pos().Y(),
          childToJoint.Pos().Z()));
    _jointPrim.CreateLocalRot1Attr().Set(pxr::GfQuatf(
          childToJoint.Rot().W(),
          childToJoint.Rot().X(),
          childToJoint.Rot().Y(),
          childToJoint.Rot().Z()));

    return true;
  }

  bool ParseSdfRevoluteJoint(const sdf::Joint &_joint,
      pxr::UsdStageRefPtr &_stage, const std::string &_path)
  {
    auto usdJoint =
      pxr::UsdPhysicsRevoluteJoint::Define(_stage, pxr::SdfPath(_path));

    const auto axis = _joint.Axis();

    if (axis->Xyz() == ignition::math::Vector3d::UnitX ||
        axis->Xyz() == -ignition::math::Vector3d::UnitX)
      usdJoint.CreateAxisAttr().Set(pxr::TfToken("X"));
    else if (axis->Xyz() == ignition::math::Vector3d::UnitY ||
        axis->Xyz() == -ignition::math::Vector3d::UnitY)
      usdJoint.CreateAxisAttr().Set(pxr::TfToken("Y"));
    else if (axis->Xyz() == ignition::math::Vector3d::UnitZ ||
        axis->Xyz() == -ignition::math::Vector3d::UnitZ)
      usdJoint.CreateAxisAttr().Set(pxr::TfToken("Z"));
    else
    {
      std::cerr << "Revolute joint [" << _joint.Name() << "] has an invalid "
                << "axis: [" << axis->Xyz() << "]\n";
      return false;
    }

    // Revolute joint limits in SDF are in radians, but USD expects degrees
    // of C++ type float
    auto sdfLimitDegrees = static_cast<float>(
        ignition::math::Angle(axis->Lower()).Degree());
    usdJoint.CreateLowerLimitAttr().Set(sdfLimitDegrees);
    sdfLimitDegrees = static_cast<float>(
        ignition::math::Angle(axis->Upper()).Degree());
    usdJoint.CreateUpperLimitAttr().Set(sdfLimitDegrees);

    pxr::UsdPrim usdJointPrim = _stage->GetPrimAtPath(pxr::SdfPath(_path));

    auto drive =
      pxr::UsdPhysicsDriveAPI::Apply(usdJointPrim, pxr::TfToken("angular"));
    if (!drive)
    {
      std::cerr << "Internal error: unable to mark link at path ["
                << _path << "] as a UsdPhysicsDriveAPI\n";
      return false;
    }

    drive.CreateDampingAttr().Set(static_cast<float>(axis->Damping()));
    drive.CreateStiffnessAttr().Set(static_cast<float>(axis->Stiffness()));
    drive.CreateMaxForceAttr().Set(static_cast<float>(axis->Effort()));

    return true;
  }

  bool ParseSdfBallJoint(const sdf::Joint &_joint,
      pxr::UsdStageRefPtr &_stage, const std::string &_path)
  {
    pxr::UsdPhysicsSphericalJoint::Define(_stage, pxr::SdfPath(_path));

    // While USD allows for cone limits that can restrict motion in a given
    // range, SDF does not have limits for a ball joint. So, there's
    // nothing to do after creating a UsdPhysicsSphericalJoint, since this
    // joint by default has no limits (i.e., allows for circular motion)
    return true;
  }

  bool ParseSdfFixedJoint(const sdf::Joint &_joint,
      pxr::UsdStageRefPtr &_stage, const std::string &_path)
  {
    pxr::UsdPhysicsFixedJoint::Define(_stage, pxr::SdfPath(_path));

    return true;
  }

  bool ParseSdfPrismaticJoint(const sdf::Joint &_joint,
      pxr::UsdStageRefPtr &_stage, const std::string &_path)
  {
    auto usdJoint =
      pxr::UsdPhysicsPrismaticJoint::Define(_stage, pxr::SdfPath(_path));

    if (_joint.Axis()->Xyz() == ignition::math::Vector3d::UnitX ||
        _joint.Axis()->Xyz() == -ignition::math::Vector3d::UnitX)
      usdJoint.CreateAxisAttr().Set(pxr::TfToken("X"));
    else if (_joint.Axis()->Xyz() == ignition::math::Vector3d::UnitY ||
        _joint.Axis()->Xyz() == -ignition::math::Vector3d::UnitY)
      usdJoint.CreateAxisAttr().Set(pxr::TfToken("Y"));
    else if (_joint.Axis()->Xyz() == ignition::math::Vector3d::UnitZ ||
        _joint.Axis()->Xyz() == -ignition::math::Vector3d::UnitZ)
      usdJoint.CreateAxisAttr().Set(pxr::TfToken("Z"));
    else
    {
      std::cerr << "Prismatic joint [" << _joint.Name() << "] has an invalid "
                << "axis: [" << _joint.Axis()->Xyz() << "]\n";
      return false;
    }

    usdJoint.CreateLowerLimitAttr().Set(
        static_cast<float>(_joint.Axis()->Lower()));
    usdJoint.CreateUpperLimitAttr().Set(
        static_cast<float>(_joint.Axis()->Upper()));

    return true;
  }

  bool ParseSdfJoint(const sdf::Joint &_joint,
      pxr::UsdStageRefPtr &_stage, const std::string &_path,
      const sdf::Model &_parentModel,
      const std::unordered_map<std::string, pxr::SdfPath> &_linkToUSDPath,
      const pxr::SdfPath &_worldPath)
  {
    // the joint's parent may be "world". If this is the case, the joint's
    // parent should be set to the world prim, not a link
    auto parentLinkPath = _worldPath;
    if (_joint.ParentLinkName() != "world")
    {
      const auto it = _linkToUSDPath.find(_joint.ParentLinkName());
      if (it == _linkToUSDPath.end())
      {
        std::cerr << "Unable to find a USD path for link ["
                  << _joint.ParentLinkName() << "], which is the parent link "
                  << "of joint [" << _joint.Name() << "]\n";
        return false;
      }
      parentLinkPath = it->second;
    }

    const auto it = _linkToUSDPath.find(_joint.ChildLinkName());
    if (it == _linkToUSDPath.end())
    {
      std::cerr << "Unable to find a USD path for link ["
                << _joint.ParentLinkName() << "], which is the child link "
                << "of joint [" << _joint.Name() << "]\n";
      return false;
    }
    const auto childLinkPath = it->second;

    bool typeParsed = false;
    switch (_joint.Type())
    {
      case sdf::JointType::REVOLUTE:
        typeParsed = ParseSdfRevoluteJoint(_joint, _stage, _path);
        break;
      case sdf::JointType::BALL:
        typeParsed = ParseSdfBallJoint(_joint, _stage, _path);
        break;
      case sdf::JointType::FIXED:
        typeParsed = ParseSdfFixedJoint(_joint, _stage, _path);
        break;
      case sdf::JointType::PRISMATIC:
        typeParsed = ParseSdfPrismaticJoint(_joint, _stage, _path);
        break;
      case sdf::JointType::CONTINUOUS:
      case sdf::JointType::GEARBOX:
      case sdf::JointType::REVOLUTE2:
      case sdf::JointType::SCREW:
      case sdf::JointType::UNIVERSAL:
      case sdf::JointType::INVALID:
      default:
        std::cerr << "Joint type is either invalid or not supported\n";
    }

    if (typeParsed)
    {
      auto jointPrim = pxr::UsdPhysicsJoint::Get(_stage, pxr::SdfPath(_path));
      if (!jointPrim)
      {
        std::cerr << "Internal error: unable to get prim at path ["
                  << _path << "], but a joint prim should exist at this path\n";
        return false;
      }

      // define the joint's parent/child links
      jointPrim.CreateBody0Rel().AddTarget(parentLinkPath);
      jointPrim.CreateBody1Rel().AddTarget(childLinkPath);

      if (!SetUSDJointPose(jointPrim, _joint, _parentModel))
      {
        std::cerr << "Unable to set the joint pose for joint ["
                  << _joint.Name() << "]\n";
        return false;
      }
    }

    return typeParsed;
  }
}
