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

#include "USDJoints.hh"

#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/usd/usdGeom/gprim.h>
#include <pxr/usd/usdPhysics/joint.h>
#include <pxr/usd/usdPhysics/fixedJoint.h>
#include <pxr/usd/usdPhysics/prismaticJoint.h>
#include <pxr/usd/usdPhysics/revoluteJoint.h>
#pragma pop_macro ("__DEPRECATED")

#include <ignition/common/Util.hh>

#include "sdf/usd/usd_parser/USDData.hh"
#include "sdf/Console.hh"
#include "sdf/Joint.hh"
#include "sdf/JointAxis.hh"

namespace sdf
{
  // Inline bracket to help doxygen filtering.
  inline namespace SDF_VERSION_NAMESPACE {
  //
  namespace usd
  {
    sdf::Joint ParseJoints(
      const pxr::UsdPrim &_prim,
      const std::string &_path,
      const USDData &_usdData)
    {
      sdf::Joint joint;

      std::pair<std::string, std::shared_ptr<USDStage>> usdData =
        _usdData.FindStage(_prim.GetPath().GetName());
      double metersPerUnit = usdData.second->MetersPerUnit();

      pxr::SdfPathVector body0, body1;

      auto variant_physics_joint = pxr::UsdPhysicsJoint(_prim);

      if (variant_physics_joint.GetBody0Rel())
        variant_physics_joint.GetBody0Rel().GetTargets(&body0);
      if (variant_physics_joint.GetBody1Rel())
        variant_physics_joint.GetBody1Rel().GetTargets(&body1);

      if (body1.size() > 0)
      {
        joint.SetChildLinkName(ignition::common::basename(
          body1[0].GetString()));
      }
      else if (body0.size() > 0)
      {
        joint.SetParentLinkName("world");
        joint.SetChildLinkName(ignition::common::basename(
          body0[0].GetString()));
      }

      if (body0.size() > 0 && joint.ParentLinkName().empty())
      {
        joint.SetParentLinkName(ignition::common::basename(
          body0[0].GetString()));
      }
      else
      {
        joint.SetParentLinkName("world");
      }

      joint.SetName(ignition::common::basename(_path) + "_joint");

      float lowerLimit;
      float upperLimit;
      float stiffness;
      float damping;
      float maxForce;
      float jointFriction;
      float vel;
      ignition::math::Quaterniond q1;
      ignition::math::Quaterniond q2;
      pxr::GfVec3f trans;
      ignition::math::Vector3d axisVector;
      sdf::JointAxis jointAxis;

      if (_prim.IsA<pxr::UsdPhysicsPrismaticJoint>() ||
          _prim.IsA<pxr::UsdPhysicsRevoluteJoint>())
      {
        joint.SetPoseRelativeTo(joint.ParentLinkName());

        pxr::TfToken axis;
        if (_prim.IsA<pxr::UsdPhysicsPrismaticJoint>())
        {
          pxr::UsdPhysicsPrismaticJoint(_prim).GetAxisAttr().Get(&axis);
        }
        else
        {
          pxr::UsdPhysicsRevoluteJoint(_prim).GetAxisAttr().Get(&axis);
        }

        if (axis == pxr::UsdGeomTokens->x)
        {
          axisVector = ignition::math::Vector3d(1, 0, 0);
        }
        else if (axis == pxr::UsdGeomTokens->y)
        {
          axisVector = ignition::math::Vector3d(0, 1, 0);
        }
        else if (axis == pxr::UsdGeomTokens->z)
        {
          axisVector = ignition::math::Vector3d(0, 0, 1);
        }

        pxr::GfVec3f localPose0, localPose1;
        pxr::GfQuatf localRot0, localRot1;

        const auto usdPhysicsJoint = pxr::UsdPhysicsJoint(_prim);
        usdPhysicsJoint.GetLocalPos0Attr().Get(&localPose0);
        usdPhysicsJoint.GetLocalPos1Attr().Get(&localPose1);
        usdPhysicsJoint.GetLocalRot0Attr().Get(&localRot0);
        usdPhysicsJoint.GetLocalRot1Attr().Get(&localRot1);

        trans = (localPose0 + localPose1) * metersPerUnit;

        q1 = ignition::math::Quaterniond(
          localRot0.GetReal(),
          localRot0.GetImaginary()[0],
          localRot0.GetImaginary()[1],
          localRot0.GetImaginary()[2]);
        q2 = ignition::math::Quaterniond(
          localRot1.GetReal(),
          localRot1.GetImaginary()[0],
          localRot1.GetImaginary()[1],
          localRot1.GetImaginary()[2]);

        _prim.GetAttribute(
          pxr::TfToken("physics:lowerLimit")).Get(&lowerLimit);
        _prim.GetAttribute(
          pxr::TfToken("physics:upperLimit")).Get(&upperLimit);
        if (_prim.IsA<pxr::UsdPhysicsPrismaticJoint>())
        {
          _prim.GetAttribute(
            pxr::TfToken("drive:linear:physics:stiffness")).Get(&stiffness);
          _prim.GetAttribute(
            pxr::TfToken("drive:linear:physics:damping")).Get(&damping);
          _prim.GetAttribute(
            pxr::TfToken("drive:linear:physics:maxForce")).Get(&maxForce);
        }
        else
        {
          _prim.GetAttribute(
            pxr::TfToken("drive:angular:physics:stiffness")).Get(&stiffness);
          _prim.GetAttribute(
            pxr::TfToken("drive:angular:physics:damping")).Get(&damping);
          _prim.GetAttribute(
            pxr::TfToken("drive:angular:physics:maxForce")).Get(&maxForce);
        }
        _prim.GetAttribute(
          pxr::TfToken("physxJoint:maxJointVelocity")).Get(&vel);

        pxr::UsdAttribute jointFrictionAttribute;
        if (jointFrictionAttribute = _prim.GetAttribute(
          pxr::TfToken("physxJoint:jointFriction")))
        {
          jointFrictionAttribute.Get(&jointFriction);
        }
        else if (jointFrictionAttribute = _prim.GetAttribute(
          pxr::TfToken("jointFriction")))
        {
          jointFrictionAttribute.Get(&jointFriction);
        }

        jointAxis.SetDamping(damping);
        jointAxis.SetEffort(maxForce);
        jointAxis.SetSpringStiffness(stiffness);
        jointAxis.SetFriction(jointFriction);
        jointAxis.SetMaxVelocity(vel);
      }

      if (_prim.IsA<pxr::UsdPhysicsFixedJoint>())
      {
        auto variant_physics_fixed_joint = pxr::UsdPhysicsFixedJoint(_prim);

        joint.SetType(sdf::JointType::FIXED);

        return joint;
      }
      else if (_prim.IsA<pxr::UsdPhysicsPrismaticJoint>())
      {
        auto variant_physics_prismatic_joint =
          pxr::UsdPhysicsPrismaticJoint(_prim);

        joint.SetType(sdf::JointType::PRISMATIC);

        auto errors = jointAxis.SetXyz(-(q2 * axisVector).Round());
        if (!errors.empty())
        {
          std::cerr << "Errors encountered when setting xyz of prismatic "
                    << "joint axis:\n";
          for (const auto &e : errors)
            std::cerr << e << "\n";
        }

        joint.SetRawPose(
          ignition::math::Pose3d(
            ignition::math::Vector3d(trans[0], trans[1], trans[2]),
            ignition::math::Quaterniond(q1 * q2)));

        jointAxis.SetLower(lowerLimit * metersPerUnit);
        jointAxis.SetUpper(upperLimit * metersPerUnit);
        joint.SetAxis(0, jointAxis);

        return joint;
      }
      else if (_prim.IsA<pxr::UsdPhysicsRevoluteJoint>())
      {
        auto variant_physics_revolute_joint = pxr::UsdPhysicsRevoluteJoint(_prim);

        joint.SetType(sdf::JointType::REVOLUTE);

        auto errors = jointAxis.SetXyz(axisVector);
        if (!errors.empty())
        {
          std::cerr << "Errors encountered when setting xyz of revolute "
                    << "joint axis:\n";
          for (const auto &e : errors)
            std::cerr << e << "\n";
        }

        joint.SetRawPose(ignition::math::Pose3d(
            ignition::math::Vector3d(trans[0], trans[1], trans[2]),
            q1));

        jointAxis.SetLower(IGN_DTOR(lowerLimit));
        jointAxis.SetUpper(IGN_DTOR(upperLimit));
        joint.SetAxis(0, jointAxis);

        return joint;
      }
      else if (_prim.IsA<pxr::UsdPhysicsJoint>())
      {
        auto variant_physics_fixed_joint = pxr::UsdPhysicsJoint(_prim);

        joint.SetType(sdf::JointType::FIXED);

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
  }
}
