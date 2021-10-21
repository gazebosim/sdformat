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

#include "sdf/Console.hh"


namespace usd
{
  JointSharedPtr ParseJoints(const pxr::UsdPrim &_prim)
  {
    JointSharedPtr joint = nullptr;
    if (_prim.IsA<pxr::UsdPhysicsFixedJoint>())
    {
      auto variant_physics_fixed_joint = pxr::UsdPhysicsFixedJoint(_prim);

      sdferr << "UsdPhysicsFixedJoint" << "\n";
      joint.reset(new Joint);

      joint->clear();

      std::string joint_name = _prim.GetName().GetText();
      joint->name = joint_name;

      joint->parent_to_joint_origin_transform.clear();
      joint->type = Joint::FIXED;

      pxr::SdfPathVector body0, body1;
      variant_physics_fixed_joint.GetBody0Rel().GetTargets(&body0);
      variant_physics_fixed_joint.GetBody1Rel().GetTargets(&body1);
      joint->parent_link_name = "/World/" + body0[0].GetName();
      joint->child_link_name = "/World/" + body1[0].GetName();
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
