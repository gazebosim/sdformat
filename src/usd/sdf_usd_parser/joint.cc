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
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>
#include <pxr/base/gf/quatf.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/tf/token.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/relationship.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdPhysics/fixedJoint.h>
#include <pxr/usd/usdPhysics/joint.h>
#include <pxr/usd/usdPhysics/prismaticJoint.h>
#include <pxr/usd/usdPhysics/revoluteJoint.h>
#include <pxr/usd/usdPhysics/sphericalJoint.h>

#include "sdf/Joint.hh"
#include "sdf/JointAxis.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"

namespace usd
{
  /// \brief Helper function for setting a USD joint's pose relative to the
  /// joint's parent and child links.
  /// \param[in] _jointPrim The USD joint prim
  /// \param[in] _joint The SDF representation of _jointPrim
  /// \param[in] _parentModel The SDF model that has the SDF definition of
  /// _joint, which is used to extract pose information from the joint's parent
  /// and child links.
  /// \return True if _joint's pose was properly set relative to the joint's
  /// parent and child links. False otherwise
  /// \note By default, SDF has the joint at the root of the child link. So,
  /// this method assumes that the USD joint should be placed at the root of the
  /// child link to mimic the default SDF behavior.
  /// TODO(adlarkin) handle joints that don't have a default pose at the root of
  /// the child link
  bool SetUSDJointPose(pxr::UsdPhysicsJoint &_jointPrim,
      const sdf::Joint &_joint, const sdf::Model &_parentModel)
  {
    auto sdfChildLink = _parentModel.LinkByName(_joint.ChildLinkName());
    if (!sdfChildLink)
    {
      std::cerr << "Internal error: unable to find a link named ["
                << _joint.ChildLinkName() << "] in model ["
                << _parentModel.Name() << "]\n";
      return false;
    }

    ignition::math::Vector3d relativePosition;
    ignition::math::Quaterniond relativeRotation;

    if (_joint.ParentLinkName() == "world")
    {
      // This joint's parent is the world, and the SDF child link pose is
      // w.r.t world by default, so no pose computations need to be done
      // since the child link pose is already w.r.t the parent
      relativePosition = sdfChildLink->RawPose().Pos();
      relativeRotation = sdfChildLink->RawPose().Rot();
    }
    else
    {
      auto sdfParentLink = _parentModel.LinkByName(_joint.ParentLinkName());
      if (!sdfParentLink)
      {
        std::cerr << "Internal error: unable to find a link named ["
                  << _joint.ParentLinkName() << "] in model ["
                  << _parentModel.Name() << "]\n";
        return false;
      }

      // Compute the child link's pose w.r.t the parent link's pose.
      //
      // Given the following frame names:
      // W: World/inertial frame
      // P: Parent link frame
      // C: Child link frame
      //
      // And the following quantities:
      // parentWorldPose (X_WP): Pose of the parent link frame w.r.t the world
      // childWorldPose (X_WC): Pose of the child link frame w.r.t the world
      // childParentPose (X_PC): Pose of the child link frame w.r.t. the parent
      //   link frame
      //
      // The pose of the child link frame w.r.t the parent link frame (X_PC)
      // is calculated as:
      //   X_PC = (X_WP)^-1 * X_WC
      const auto &x_wp = sdfParentLink->RawPose();
      const auto &x_wc = sdfChildLink->RawPose();
      const auto &childParentPose = x_wp.Inverse() * x_wc;

      relativePosition = childParentPose.Pos();
      relativeRotation = childParentPose.Rot();
    }


    // set the joint's pose relative to the parent link to be at the root of
    // the child link (see \note in the method documentation above)
    _jointPrim.CreateLocalPos0Attr().Set(pxr::GfVec3f(
          relativePosition.X(), relativePosition.Y(), relativePosition.Z()));
    _jointPrim.CreateLocalRot0Attr().Set(pxr::GfQuatf(
          relativeRotation.W(), relativeRotation.X(),
          relativeRotation.Y(), relativeRotation.Z()));

    // since we are currently assuming that the joint is attached to the root
    // of the child link (see the \note in the method documentation above),
    // the position and rotation offset relative to the child link is 0
    _jointPrim.CreateLocalPos1Attr().Set(pxr::GfVec3f(0.0));
    _jointPrim.CreateLocalRot1Attr().Set(pxr::GfQuatf(1.0, 0.0, 0.0, 0.0));

    return true;
  }

  bool ParseSdfRevoluteJoint(const sdf::Joint &_joint,
      pxr::UsdStageRefPtr &_stage, const std::string &_path)
  {
    auto usdJoint =
      pxr::UsdPhysicsRevoluteJoint::Define(_stage, pxr::SdfPath(_path));

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
      std::cerr << "Revolute joint [" << _joint.Name() << "] has an invalid "
                << "axis: [" << _joint.Axis()->Xyz() << "]\n";
      return false;
    }

    // Revolute joint limits in SDF are in radians, but USD expects degrees
    // of C++ type float
    auto sdfLimitDegrees = static_cast<float>(
        ignition::math::Angle(_joint.Axis()->Lower()).Degree());
    usdJoint.CreateLowerLimitAttr().Set(sdfLimitDegrees);
    sdfLimitDegrees = static_cast<float>(
        ignition::math::Angle(_joint.Axis()->Upper()).Degree());
    usdJoint.CreateUpperLimitAttr().Set(sdfLimitDegrees);

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

    if (_joint.Axis()->Xyz() == ignition::math::Vector3d::UnitX)
      usdJoint.CreateAxisAttr().Set(pxr::TfToken("X"));
    else if (_joint.Axis()->Xyz() == ignition::math::Vector3d::UnitY)
      usdJoint.CreateAxisAttr().Set(pxr::TfToken("Y"));
    else if (_joint.Axis()->Xyz() == ignition::math::Vector3d::UnitZ)
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
