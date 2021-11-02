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

#include "joints.hh"

#include "pxr/usd/usdPhysics/scene.h"
#include "pxr/usd/usdPhysics/joint.h"
#include "pxr/usd/usdPhysics/fixedJoint.h"
#include "pxr/usd/usdPhysics/prismaticJoint.h"
#include "pxr/usd/usdPhysics/revoluteJoint.h"

#include "sdf/Console.hh"
#include "sdf/Joint.hh"
#include "sdf/JointAxis.hh"


namespace usd
{
  std::shared_ptr<sdf::Joint> ParseJoints(const pxr::UsdPrim &_prim, const std::string &_path,
    const double _metersPerUnit)
  {
    std::cerr << "************ ADDED JOINT ************" << '\n';
    std::shared_ptr<sdf::Joint> joint = nullptr;
    joint = std::make_shared<sdf::Joint>();

    pxr::SdfPathVector body0, body1;

    auto variant_physics_joint = pxr::UsdPhysicsJoint(_prim);

    if (variant_physics_joint.GetBody0Rel())
      variant_physics_joint.GetBody0Rel().GetTargets(&body0);
    if (variant_physics_joint.GetBody1Rel())
      variant_physics_joint.GetBody1Rel().GetTargets(&body1);

    if (body1.size() > 0)
    {
      joint->SetChildLinkName(body1[0].GetString());
    }
    else
    {
      if (body0.size() > 0)
      {
        joint->SetParentLinkName("world");
        joint->SetChildLinkName(body0[0].GetString());
      }
    }

    if (body0.size() > 0 && joint->ParentLinkName().empty())
    {
      joint->SetParentLinkName(body0[0].GetString());
    }
    else
    {
      joint->SetParentLinkName("world");
    }

    joint->SetName(_path + "_joint");

    if (_prim.IsA<pxr::UsdPhysicsFixedJoint>())
    {
      auto variant_physics_fixed_joint = pxr::UsdPhysicsFixedJoint(_prim);

      sdferr << "UsdPhysicsFixedJoint" << "\n";

      std::string joint_name = _prim.GetName().GetText();

      joint->SetType(sdf::JointType::FIXED);

      return joint;
    }
    else if (_prim.IsA<pxr::UsdPhysicsPrismaticJoint>())
    {
      auto variant_physics_prismatic_joint = pxr::UsdPhysicsPrismaticJoint(_prim);
      sdferr << "UsdPhysicsPrismaticJoint" << "\n";

      std::string joint_name = _prim.GetName().GetText();
      joint->SetType(sdf::JointType::PRISMATIC);

      pxr::TfToken axis;
      sdf::JointAxis jointAxis;
      variant_physics_prismatic_joint.GetAxisAttr().Get(&axis);
      if (axis == pxr::UsdGeomTokens->x)
      {
        jointAxis.SetXyz(ignition::math::Vector3d(1, 0, 0));
      }
      if (axis == pxr::UsdGeomTokens->y)
      {
        jointAxis.SetXyz(ignition::math::Vector3d(0, 1, 0));
      }
      if (axis == pxr::UsdGeomTokens->z)
      {
        jointAxis.SetXyz(ignition::math::Vector3d(0, 0, 1));
      }

      pxr::GfVec3f localPose0, localPose1;
      pxr::GfQuatf localRot0, localRot1;

      variant_physics_prismatic_joint.GetLocalPos0Attr().Get(&localPose0);
      variant_physics_prismatic_joint.GetLocalPos1Attr().Get(&localPose1);
      variant_physics_prismatic_joint.GetLocalRot0Attr().Get(&localRot0);
      variant_physics_prismatic_joint.GetLocalRot1Attr().Get(&localRot1);

      auto trans = (localPose0 + localPose1) * _metersPerUnit;
      joint->SetRawPose(
        ignition::math::Pose3d(
          ignition::math::Vector3d(trans[0], trans[1], trans[2]),
          ignition::math::Quaterniond(
            localRot0.GetImaginary()[0] + localRot1.GetImaginary()[0],
            localRot0.GetImaginary()[1] + localRot1.GetImaginary()[1],
            localRot0.GetImaginary()[2] + localRot1.GetImaginary()[2],
            localRot0.GetReal() + localRot1.GetReal())));

      float lowerLimit;
      float upperLimit;
      _prim.GetAttribute(pxr::TfToken("physics:lowerLimit")).Get(&lowerLimit);
      _prim.GetAttribute(pxr::TfToken("physics:upperLimit")).Get(&upperLimit);

      jointAxis.SetLower(lowerLimit * _metersPerUnit);
      jointAxis.SetUpper(upperLimit * _metersPerUnit);

      joint->SetAxis(0, jointAxis);

      return joint;
    }
    else if (_prim.IsA<pxr::UsdPhysicsRevoluteJoint>())
    {
      auto variant_physics_revolute_joint = pxr::UsdPhysicsRevoluteJoint(_prim);
      sdferr << "UsdPhysicsRevoluteJoint" << "\n";

      std::string joint_name = _prim.GetName().GetText();
      joint->SetType(sdf::JointType::REVOLUTE);

      sdf::JointAxis jointAxis;
      pxr::TfToken axis;
      variant_physics_revolute_joint.GetAxisAttr().Get(&axis);
      if (axis == pxr::UsdGeomTokens->x)
      {
        jointAxis.SetXyz(ignition::math::Vector3d(1, 0, 0));
      }
      if (axis == pxr::UsdGeomTokens->y)
      {
        jointAxis.SetXyz(ignition::math::Vector3d(0, 1, 0));
      }
      if (axis == pxr::UsdGeomTokens->z)
      {
        jointAxis.SetXyz(ignition::math::Vector3d(0, 0, 1));
      }

      pxr::GfVec3f localPose0, localPose1;
      pxr::GfQuatf localRot0, localRot1;

      variant_physics_revolute_joint.GetLocalPos0Attr().Get(&localPose0);
      variant_physics_revolute_joint.GetLocalPos1Attr().Get(&localPose1);
      variant_physics_revolute_joint.GetLocalRot0Attr().Get(&localRot0);
      variant_physics_revolute_joint.GetLocalRot1Attr().Get(&localRot1);

      auto trans = (localPose0 + localPose1) * _metersPerUnit;
      joint->SetRawPose(
        ignition::math::Pose3d(
          ignition::math::Vector3d(trans[0], trans[1], trans[2]),
          ignition::math::Quaterniond(
            localRot0.GetImaginary()[0] + localRot1.GetImaginary()[0],
            localRot0.GetImaginary()[1] + localRot1.GetImaginary()[1],
            localRot0.GetImaginary()[2] + localRot1.GetImaginary()[2],
            localRot0.GetReal() + localRot1.GetReal())));

      float lowerLimit;
      float upperLimit;
      float stiffness;
      float damping;
      float jointFriction;
      _prim.GetAttribute(pxr::TfToken("physics:lowerLimit")).Get(&lowerLimit);
      _prim.GetAttribute(pxr::TfToken("physics:upperLimit")).Get(&upperLimit);
      _prim.GetAttribute(pxr::TfToken("drive:angular:physics:stiffness")).Get(&stiffness);
      _prim.GetAttribute(pxr::TfToken("drive:angular:physics:damping")).Get(&damping);
      _prim.GetAttribute(pxr::TfToken("physxJoint:jointFriction")).Get(&jointFriction);

      jointAxis.SetLower(lowerLimit * 3.1416 / 180.0);
      jointAxis.SetUpper(upperLimit * 3.1416 / 180.0);
      jointAxis.SetEffort(stiffness);

      jointAxis.SetDamping(damping);
      jointAxis.SetFriction(jointFriction);

      joint->SetAxis(0, jointAxis);

      return joint;
    }
    //limtis

    // Get safety

    // Get Dynamics

    // Get Mimics

    // Get calibration

    return joint;
  }
}
