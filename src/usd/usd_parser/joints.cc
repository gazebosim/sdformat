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

#include <pxr/usd/usdGeom/cube.h>
#include "pxr/usd/usdGeom/gprim.h"
#include <pxr/usd/usdGeom/cylinder.h>

#include "ignition/common/Util.hh"

#include "sdf/Console.hh"
#include "sdf/Joint.hh"
#include "sdf/JointAxis.hh"

#include "utils.hh"

namespace usd
{
  std::shared_ptr<sdf::Joint> ParseJoints(
    const pxr::UsdPrim &_prim,
    const std::string &_path,
    USDData &_usdData)
  {
    std::cerr << "************ ADDED JOINT ************" << '\n';
    std::shared_ptr<sdf::Joint> joint = nullptr;
    joint = std::make_shared<sdf::Joint>();

    std::pair<std::string, std::shared_ptr<USDStage>> lightUSDData =
      _usdData.findStage(_prim.GetPath().GetName());

    double metersPerUnit = lightUSDData.second->_metersPerUnit;

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
      ignition::math::Vector3d v;
      if (axis == pxr::UsdGeomTokens->x)
      {
        v = ignition::math::Vector3d(1, 0, 0);
      }
      if (axis == pxr::UsdGeomTokens->y)
      {
        v = ignition::math::Vector3d(0, 1, 0);
      }
      if (axis == pxr::UsdGeomTokens->z)
      {
        v = ignition::math::Vector3d(0, 0, 1);
      }

      pxr::GfVec3f localPose0, localPose1;
      pxr::GfQuatf localRot0, localRot1;

      variant_physics_prismatic_joint.GetLocalPos0Attr().Get(&localPose0);
      variant_physics_prismatic_joint.GetLocalPos1Attr().Get(&localPose1);
      variant_physics_prismatic_joint.GetLocalRot0Attr().Get(&localRot0);
      variant_physics_prismatic_joint.GetLocalRot1Attr().Get(&localRot1);

      auto trans = (localPose0 + localPose1) * metersPerUnit;

      ignition::math::Quaterniond q1(
        localRot0.GetReal(),
        localRot0.GetImaginary()[0],
        localRot0.GetImaginary()[1],
        localRot0.GetImaginary()[2]);
      ignition::math::Quaterniond q2(
        localRot1.GetReal(),
        localRot1.GetImaginary()[0],
        localRot1.GetImaginary()[1],
        localRot1.GetImaginary()[2]);

      std::cerr << "============================== " << q2 * v << '\n';

      auto errors = jointAxis.SetXyz(-(q2 * v).Round());
      if (!errors.empty())
      {
        std::cerr << "Errors encountered when setting xyz of prismatic "
                  << "joint axis:\n";
        for (const auto &e : errors)
          std::cerr << e << "\n";
      }

      joint->SetRawPose(
        ignition::math::Pose3d(
          ignition::math::Vector3d(trans[0], trans[1], trans[2]),
          ignition::math::Quaterniond(q1 * q2)));

      float lowerLimit;
      float upperLimit;
      float stiffness;
      float damping;
      float maxForce;
      float jointFriction;
      float vel;

      _prim.GetAttribute(pxr::TfToken("physics:lowerLimit")).Get(&lowerLimit);
      _prim.GetAttribute(pxr::TfToken("physics:upperLimit")).Get(&upperLimit);
      _prim.GetAttribute(pxr::TfToken("drive:linear:physics:stiffness")).Get(&stiffness);
      _prim.GetAttribute(pxr::TfToken("drive:linear:physics:damping")).Get(&damping);
      _prim.GetAttribute(pxr::TfToken("drive:linear:physics:maxForce")).Get(&maxForce);
      _prim.GetAttribute(pxr::TfToken("physxJoint:maxJointVelocity")).Get(&vel);

      pxr::UsdAttribute jointFrictionAttribute;
      if (jointFrictionAttribute = _prim.GetAttribute(pxr::TfToken("physxJoint:jointFriction")))
      {
        jointFrictionAttribute.Get(&jointFriction);
      }
      else if (jointFrictionAttribute = _prim.GetAttribute(pxr::TfToken("jointFriction")))
      {
        jointFrictionAttribute.Get(&jointFriction);
      }

      jointAxis.SetLower(lowerLimit * metersPerUnit);
      jointAxis.SetUpper(upperLimit * metersPerUnit);
      jointAxis.SetDamping(damping);
      jointAxis.SetEffort(maxForce);
      jointAxis.SetSpringStiffness(stiffness);
      jointAxis.SetFriction(jointFriction);
      jointAxis.SetMaxVelocity(vel);

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
      ignition::math::Vector3d v;
      if (axis == pxr::UsdGeomTokens->x)
      {
        v = ignition::math::Vector3d(1, 0, 0);
      }
      if (axis == pxr::UsdGeomTokens->y)
      {
        v = ignition::math::Vector3d(0, 1, 0);
      }
      if (axis == pxr::UsdGeomTokens->z)
      {
        v = ignition::math::Vector3d(0, 0, 1);
      }

      pxr::GfVec3f localPose0, localPose1;
      pxr::GfQuatf localRot0, localRot1;

      variant_physics_revolute_joint.GetLocalPos0Attr().Get(&localPose0);
      variant_physics_revolute_joint.GetLocalPos1Attr().Get(&localPose1);
      variant_physics_revolute_joint.GetLocalRot0Attr().Get(&localRot0);
      variant_physics_revolute_joint.GetLocalRot1Attr().Get(&localRot1);

      auto trans = (localPose0 + localPose1) * metersPerUnit;
      std::cerr << "trans " << trans << '\n';
      // exit(-1);
      ignition::math::Quaterniond q1(
        localRot0.GetReal(),
        localRot0.GetImaginary()[0],
        localRot0.GetImaginary()[1],
        localRot0.GetImaginary()[2]);
      ignition::math::Quaterniond q2(
        localRot1.GetReal(),
        localRot1.GetImaginary()[0],
        localRot1.GetImaginary()[1],
        localRot1.GetImaginary()[2]);

      std::cerr << "============================== " << q2 * v << '\n';

      auto errors = jointAxis.SetXyz(v);
      if (!errors.empty())
      {
        std::cerr << "Errors encountered when setting xyz of revolute "
                  << "joint axis:\n";
        for (const auto &e : errors)
          std::cerr << e << "\n";
      }

      joint->SetRawPose(
        ignition::math::Pose3d(
          ignition::math::Vector3d(trans[0], trans[1], trans[2]),
          ignition::math::Quaterniond(q1)));

      float lowerLimit;
      float upperLimit;
      float stiffness;
      float damping;
      float jointFriction;
      float maxForce;
      float vel;
      _prim.GetAttribute(pxr::TfToken("physics:lowerLimit")).Get(&lowerLimit);
      _prim.GetAttribute(pxr::TfToken("physics:upperLimit")).Get(&upperLimit);
      _prim.GetAttribute(pxr::TfToken("drive:angular:physics:stiffness")).Get(&stiffness);
      _prim.GetAttribute(pxr::TfToken("drive:angular:physics:damping")).Get(&damping);
      _prim.GetAttribute(pxr::TfToken("drive:angular:physics:maxForce")).Get(&maxForce);
      _prim.GetAttribute(pxr::TfToken("physxJoint:maxJointVelocity")).Get(&vel);

      pxr::UsdAttribute jointFrictionAttribute;
      if (jointFrictionAttribute = _prim.GetAttribute(pxr::TfToken("physxJoint:jointFriction")))
      {
        jointFrictionAttribute.Get(&jointFriction);
      }
      else if (jointFrictionAttribute = _prim.GetAttribute(pxr::TfToken("jointFriction")))
      {
        jointFrictionAttribute.Get(&jointFriction);
      }

      jointAxis.SetLower(lowerLimit * 3.1416 / 180.0);
      jointAxis.SetUpper(upperLimit * 3.1416 / 180.0);
      jointAxis.SetEffort(maxForce);

      jointAxis.SetSpringStiffness(stiffness);

      jointAxis.SetDamping(damping);
      jointAxis.SetFriction(jointFriction);
      jointAxis.SetMaxVelocity(vel);

      joint->SetAxis(0, jointAxis);

      return joint;
    }
    else if (_prim.IsA<pxr::UsdPhysicsJoint>())
    {
      auto variant_physics_fixed_joint = pxr::UsdPhysicsJoint(_prim);

      sdferr << "UsdPhysicsJoint" << "\n";

      std::string joint_name = _prim.GetName().GetText();

      joint->SetType(sdf::JointType::FIXED);

      return joint;
    }
    //limtis

    // Get safety

    // Get Dynamics

    // Get Mimics

    // Get calibration

    return joint;
  }

  std::pair<std::shared_ptr<sdf::Joint>, LinkSharedPtr>
    ParseVehicleJoints(
      const pxr::UsdPrim &_prim,
      const std::string &_path,
      USDData &_usdData)
  {
    std::cerr << "************ ADDED VEHICLE JOINT ************" << '\n';
    LinkSharedPtr link = nullptr;
    std::shared_ptr<sdf::Joint> joint = nullptr;

    joint = std::make_shared<sdf::Joint>();
    link.reset(new Link);
    link->clear();

    std::pair<std::string, std::shared_ptr<USDStage>> lightUSDData =
      _usdData.findStage(_prim.GetPath().GetName());

    double metersPerUnit = lightUSDData.second->_metersPerUnit;

    std::string primName = pxr::TfStringify(_prim.GetPath());
    size_t pos = std::string::npos;
    if ((pos  = primName.find("/World") )!= std::string::npos)
    {
      primName.erase(pos, std::string("/World").length());
    }

    link->name = primName;
    std::cerr << "link->name " << link->name << '\n';

    joint->SetName(_path + "_joint");

    // Inertial
    {
      ignition::math::MassMatrix3d massMatrix;
      link->inertial = std::make_shared<ignition::math::Inertiald>();
      float mass;
      pxr::GfVec3f centerOfMass;

      _prim.GetAttribute(pxr::TfToken("physxVehicleWheel:mass")).Get(&mass);
      _prim.GetAttribute(
        pxr::TfToken("physxVehicleWheelAttachment:wheelCenterOfMassOffset")).Get(&centerOfMass);

      std::cerr << "mass " << mass << '\n';
      std::cerr << "centerOfMass " << centerOfMass << '\n';

      massMatrix.SetMass(mass);
      link->inertial->SetMassMatrix(massMatrix);

      link->inertial->SetPose(ignition::math::Pose3d(
          ignition::math::Vector3d(
            centerOfMass[0], centerOfMass[1], centerOfMass[2]) * metersPerUnit,
          ignition::math::Quaterniond(1.0, 0, 0, 0)));
    }

    pxr::TfTokenVector schemas = _prim.GetAppliedSchemas();
    bool isPhysxVehicleWheelAPI = false;
    for (auto & token : schemas)
    {
      std::cerr << "GetText " << token.GetText() << '\n';
      if (std::string(token.GetText()) == "PhysxVehicleWheelAPI")
      {
        isPhysxVehicleWheelAPI = true;
      }
    }

    if (isPhysxVehicleWheelAPI)
    {
      auto variant_geom = pxr::UsdGeomGprim(_prim);

      Transforms t = ParseTransform(_prim, _usdData);
      // std::tuple<pxr::GfVec3f, pxr::GfVec3f, pxr::GfQuatf, bool, bool, bool, bool> transformsTuple =
      //   ParseTransform(_prim);
      //
      // pxr::GfVec3f scale = std::get<0>(transformsTuple);
      // pxr::GfVec3f translate = std::get<1>(transformsTuple);
      // pxr::GfQuatf rotation_quad = std::get<2>(transformsTuple);
      // bool isScale = std::get<3>(transformsTuple);

      std::string joint_name = _prim.GetName().GetText();
      joint->SetType(sdf::JointType::CONTINUOUS);

      sdf::JointAxis jointAxis;
      pxr::TfToken axis;

      auto errors = jointAxis.SetXyz(ignition::math::Vector3d(0, 1, 0));
      if (!errors.empty())
      {
        std::cerr << "Errors encountered when setting xyz of vehicle "
                  << "joint axis:\n";
        for (const auto &e : errors)
          std::cerr << e << "\n";
      }

      joint->SetRawPose(
        ignition::math::Pose3d(t.translate * metersPerUnit,
          ignition::math::Quaterniond(t.q[0])));

      std::string childName;
      std::string parentName = pxr::TfStringify(_prim.GetPath().GetParentPath());
      for (const auto & child : _prim.GetChildren())
      {
        if (child.IsA<pxr::UsdGeomGprim>())
        {
          childName = pxr::TfStringify(_prim.GetPath());

          sdf::Geometry geom;
          if (child.IsA<pxr::UsdGeomCylinder>())
          {
            std::shared_ptr<sdf::Visual> vis;
            vis = std::make_shared<sdf::Visual>();

            link->pose.Pos().X() = joint->RawPose().Pos().X();
            link->pose.Pos().Y() = joint->RawPose().Pos().Y();
            link->pose.Pos().Z() = joint->RawPose().Pos().Z();

            auto rot = joint->RawPose().Rot();
            rot *= ignition::math::Quaterniond(0.7071068, 0.7071068, 0, 0);
            link->pose.Rot().X() = rot.X();
            link->pose.Rot().Y() = rot.Y();
            link->pose.Rot().Z() = rot.Z();
            link->pose.Rot().W() = rot.W();

            sdf::Material material = ParseMaterial(_prim);

            auto variant_cylinder = pxr::UsdGeomCylinder(child);
            double radius;
            double height;
            variant_cylinder.GetRadiusAttr().Get(&radius);
            variant_cylinder.GetHeightAttr().Get(&height);

            sdf::Cylinder c;
            geom.SetType(sdf::GeometryType::CYLINDER);
            c.SetRadius(radius * metersPerUnit);
            c.SetLength(height * metersPerUnit);
            geom.SetCylinderShape(c);
            vis->SetName("visual_cylinder");
            vis->SetGeom(geom);
            vis->SetMaterial(material);
            link->visual_array.push_back(vis);

            sdf::Collision col;

            std::string collisionName = link->name + "Collision";
            col.SetName(collisionName);
            col.SetGeom(geom);

            std::shared_ptr<sdf::Collision> colPtr = std::make_shared<sdf::Collision>();
            colPtr->SetRawPose(col.RawPose());
            colPtr->SetGeom(geom);

            if (child.IsA<pxr::UsdGeomCube>())
            {
              const sdf::Box * box = colPtr->Geom()->BoxShape();
              sdf::Box * boxEditable = const_cast<sdf::Box*>(box);
              ignition::math::Vector3d size = box->Size();
              boxEditable->SetSize(t.scale);
            }

            colPtr->SetName(col.Name());
            link->collision_array.push_back(colPtr);
            // Collision (optional)
            // Assign the first collision to the .collision ptr, if it exists
            if (!link->collision_array.empty())
              link->collision = link->collision_array[0];
          }

        }
      }

      pos = std::string::npos;
      if ((pos  = parentName.find("/World") )!= std::string::npos)
      {
        parentName.erase(pos, std::string("/World").length());
      }

      if ((pos  = childName.find("/World") )!= std::string::npos)
      {
        childName.erase(pos, std::string("/World").length());
      }

      std::cerr << "parentName " << parentName << '\n';
      std::cerr << "childName " << childName << '\n';

      joint->SetParentLinkName(parentName);
      joint->SetChildLinkName(childName);

      joint->SetAxis(0, jointAxis);
    }
    return std::make_pair(joint, link);
  }
}
