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

#include "USDLinks.hh"

#include <string>

#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include "pxr/usd/usdPhysics/rigidBodyAPI.h"
#pragma pop_macro ("__DEPRECATED")

#include <ignition/math/Inertial.hh>

#include "sdf/Link.hh"

#include "sdf/usd/usd_parser/USDTransforms.hh"

namespace sdf
{
// Inline bracket to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
//
namespace usd
{
void GetInertial(
  const pxr::UsdPrim &_prim,
  ignition::math::Inertiald &_inertial)
{
  float mass;
  pxr::GfVec3f centerOfMass;
  pxr::GfVec3f diagonalInertia;

  ignition::math::MassMatrix3d massMatrix;

  if (_prim.HasAPI<pxr::UsdPhysicsRigidBodyAPI>())
  {
    if (pxr::UsdAttribute massAttribute =
      _prim.GetAttribute(pxr::TfToken("physics:mass")))
    {
      massAttribute.Get(&mass);

      if (pxr::UsdAttribute centerOfMassAttribute =
        _prim.GetAttribute(pxr::TfToken("physics:centerOfMass")))
      {
        centerOfMassAttribute.Get(&centerOfMass);
      }

      if (pxr::UsdAttribute diagonalInertiaAttribute =
        _prim.GetAttribute(pxr::TfToken("physics:diagonalInertia")))
      {
        diagonalInertiaAttribute.Get(&diagonalInertia);
      }

      // Added a diagonal inertia to avoid crash with the physics engine
      if (diagonalInertia == pxr::GfVec3f(0, 0, 0))
      {
        diagonalInertia = pxr::GfVec3f(0.0001, 0.0001, 0.0001);
      }

      if (mass < 0.0001)
      {
        mass = 0.1;
      }

      massMatrix.SetMass(mass);
      massMatrix.SetDiagonalMoments(
        ignition::math::Vector3d(
          diagonalInertia[0],
          diagonalInertia[1],
          diagonalInertia[2]));

      _inertial.SetPose(ignition::math::Pose3d(
          ignition::math::Vector3d(
            centerOfMass[0], centerOfMass[1], centerOfMass[2]),
          ignition::math::Quaterniond(1.0, 0, 0, 0)));

      _inertial.SetMassMatrix(massMatrix);
    }
  }
  else
  {
    auto parent = _prim.GetParent();
    if (parent)
    {
      GetInertial(parent, _inertial);
    }
  }
}

sdf::Link * ParseUSDLinks(
  const pxr::UsdPrim &_prim,
  const std::string &_nameLink,
  sdf::Link *_link,
  USDData &_usdData,
  int &/*_skipPrim*/)
{
  // Is this a new link?
  if (_link == nullptr)
  {
    _link = new sdf::Link();
    _link->SetName(_nameLink);

    ignition::math::Pose3d pose;
    ignition::math::Vector3d scale(1, 1, 1);
    GetTransform(_prim, _usdData, pose, scale, "");
    size_t nSlash = std::count(_nameLink.begin(), _nameLink.end(), '/');
    if (nSlash > 1)
      _link->SetRawPose(pose);
    // _link->SetScale(scale);
  }

  // If the schema is a rigid body use this name instead.
  if (_prim.HasAPI<pxr::UsdPhysicsRigidBodyAPI>())
  {
    _link->SetName(pxr::TfStringify(_prim.GetPath()));
  }

  ignition::math::Inertiald noneInertial = {{1.0,
            ignition::math::Vector3d::One, ignition::math::Vector3d::Zero},
            ignition::math::Pose3d::Zero};
  const auto inertial = _link->Inertial();
  if (inertial == noneInertial)
  {
    ignition::math::Inertiald newInertial;
    GetInertial(_prim, newInertial);
    _link->SetInertial(newInertial);
  }

  return _link;
}
}
}
}
