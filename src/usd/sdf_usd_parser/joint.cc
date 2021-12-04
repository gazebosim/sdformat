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
#include <ignition/math/Vector3.hh>
#include <pxr/base/tf/token.h>
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/relationship.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdPhysics/revoluteJoint.h>

#include "sdf/Joint.hh"
#include "sdf/JointAxis.hh"

namespace usd
{
  bool ParseSdfRevoluteJoint(const sdf::Joint &_joint,
      pxr::UsdStageRefPtr &_stage, const std::string &_path,
      const pxr::SdfPath &_parentLinkPath, const pxr::SdfPath &_childLinkPath)
  {
    auto usdJoint =
      pxr::UsdPhysicsRevoluteJoint::Define(_stage, pxr::SdfPath(_path));

    usdJoint.CreateBody0Rel().AddTarget(_parentLinkPath);
    usdJoint.CreateBody1Rel().AddTarget(_childLinkPath);

    if (_joint.Axis()->Xyz() == ignition::math::Vector3d::UnitX)
      usdJoint.CreateAxisAttr().Set(pxr::TfToken("X"));
    else if (_joint.Axis()->Xyz() == ignition::math::Vector3d::UnitY)
      usdJoint.CreateAxisAttr().Set(pxr::TfToken("Y"));
    else if (_joint.Axis()->Xyz() == ignition::math::Vector3d::UnitZ)
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

  bool ParseSdfJoint(const sdf::Joint &_joint,
      pxr::UsdStageRefPtr &_stage, const std::string &_path,
      const std::unordered_map<std::string, pxr::SdfPath> &_linkToUSDPath)
  {
    auto it = _linkToUSDPath.find(_joint.ParentLinkName());
    if (it == _linkToUSDPath.end())
    {
      std::cerr << "Unable to find a USD path for link ["
                << _joint.ParentLinkName() << "], which is the parent link "
                << "of joint [" << _joint.Name() << "]\n";
      return false;
    }
    const auto parentLinkPath = it->second;

    it = _linkToUSDPath.find(_joint.ChildLinkName());
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
        typeParsed = ParseSdfRevoluteJoint(_joint, _stage, _path,
            parentLinkPath, childLinkPath);
        break;
      case sdf::JointType::BALL:
      case sdf::JointType::CONTINUOUS:
      case sdf::JointType::FIXED:
      case sdf::JointType::GEARBOX:
      case sdf::JointType::PRISMATIC:
      case sdf::JointType::REVOLUTE2:
      case sdf::JointType::SCREW:
      case sdf::JointType::UNIVERSAL:
      case sdf::JointType::INVALID:
      default:
        std::cerr << "Joint type is either invalid or not supported\n";
    }

    return typeParsed;
  }
}
