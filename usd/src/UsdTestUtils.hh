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
#ifndef SDF_PARSER_USDTESTUTILS_HH_
#define SDF_PARSER_USDTESTUTILS_HH_

#include <gtest/gtest.h>
#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>

// TODO(ahcorde):this is to remove deprecated "warnings" in usd, these warnings
// are reported using #pragma message so normal diagnostic flags cannot remove
// them. This workaround requires this block to be used whenever usd is
// included.
#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/base/gf/vec3d.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/tf/token.h>
#include <pxr/usd/usd/attribute.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usdPhysics/rigidBodyAPI.h>
#pragma pop_macro ("__DEPRECATED")

#include "sdf/system_util.hh"

namespace sdf
{
// Inline bracke to help doxygen filtering.
inline namespace SDF_VERSION_NAMESPACE {
//
namespace usd
{
namespace testing
{

/// \brief Compare the pose of a USD prim to a desired pose
/// \param[in] _usdPrim The USD prim
/// \param[in] _targetPose The pose that _usdPrim should have
void CheckPrimPose(const pxr::UsdPrim &_usdPrim,
            const ignition::math::Pose3d &_targetPose)
{
  bool checkedTranslate = false;
  if (auto translateAttr =
      _usdPrim.GetAttribute(pxr::TfToken("xformOp:translate")))
  {
    pxr::GfVec3d usdTranslation;
    translateAttr.Get(&usdTranslation);
    EXPECT_DOUBLE_EQ(usdTranslation[0], _targetPose.Pos().X());
    EXPECT_DOUBLE_EQ(usdTranslation[1], _targetPose.Pos().Y());
    EXPECT_DOUBLE_EQ(usdTranslation[2], _targetPose.Pos().Z());
    checkedTranslate = true;
  }
  EXPECT_TRUE(checkedTranslate);

  bool checkedRotate = false;
  if (auto rotateAttr =
      _usdPrim.GetAttribute(pxr::TfToken("xformOp:rotateXYZ")))
  {
    pxr::GfVec3f usdRotation;
    rotateAttr.Get(&usdRotation);
    // USD uses degrees, but SDF uses radians. USD also uses floats for angles
    // here, but SDF uses doubles
    const auto sdfRollAngle = static_cast<float>(
        ignition::math::Angle(_targetPose.Rot().Roll()).Degree());
    EXPECT_FLOAT_EQ(usdRotation[0], sdfRollAngle);
    const auto sdfPitchAngle = static_cast<float>(
        ignition::math::Angle(_targetPose.Rot().Pitch()).Degree());
    EXPECT_FLOAT_EQ(usdRotation[1], sdfPitchAngle);
    const auto sdfYawAngle = static_cast<float>(
        ignition::math::Angle(_targetPose.Rot().Yaw()).Degree());
    EXPECT_FLOAT_EQ(usdRotation[2], sdfYawAngle);
    checkedRotate = true;
  }
  EXPECT_TRUE(checkedRotate);

  bool checkedOpOrder = false;
  if (auto opOrderAttr = _usdPrim.GetAttribute(pxr::TfToken("xformOpOrder")))
  {
    pxr::VtArray<pxr::TfToken> opNames;
    opOrderAttr.Get(&opNames);
    // TODO(adlarkin) handle things like scale in the opOrder
    // (checking for scale should be done elsehwere since prims aren't always
    // scaled, but maybe what I can do here is make sure the opNames size is
    // at least 2 and then make sure translate occurs before rotate)
    ASSERT_EQ(2u, opNames.size());
    EXPECT_EQ(pxr::TfToken("xformOp:translate"), opNames[0]);
    EXPECT_EQ(pxr::TfToken("xformOp:rotateXYZ"), opNames[1]);
    checkedOpOrder = true;
  }
  EXPECT_TRUE(checkedOpOrder);
}

/// \brief Compare the Inertial of a USD prim to the desired values
/// \param[in] _usdPrim The USD prim
/// \param[in] _targetMass Mass of the link that _usdPrim should have
/// \param[in] _targetDiagonalInertia Diagonal Inertia that _usdPrim should have
/// \param[in] _targetCenterOfMass Center of mass that _usdPrim should have
/// \param[in] _isRigid True if _usdPrim should be a rigid body, False otherwise
void CheckInertial(const pxr::UsdPrim &_usdPrim,
    float _targetMass,
    const pxr::GfVec3f &_targetDiagonalInertia,
    const pxr::GfVec3f &_targetCenterOfMass,
    bool _isRigid)
{
  bool checkedMass = false;
  if (auto massAttr =
      _usdPrim.GetAttribute(pxr::TfToken("physics:mass")))
  {
    float massUSD;
    massAttr.Get(&massUSD);
    EXPECT_FLOAT_EQ(_targetMass, massUSD);
    checkedMass = true;
  }
  EXPECT_TRUE(checkedMass);

  bool checkedDiagInertia = false;
  if (auto diagInertiaAttr =
      _usdPrim.GetAttribute(pxr::TfToken("physics:diagonalInertia")))
  {
    pxr::GfVec3f diagonalInertiaUSD;
    diagInertiaAttr.Get(&diagonalInertiaUSD);
    EXPECT_FLOAT_EQ(_targetDiagonalInertia[0], diagonalInertiaUSD[0]);
    EXPECT_FLOAT_EQ(_targetDiagonalInertia[1], diagonalInertiaUSD[1]);
    EXPECT_FLOAT_EQ(_targetDiagonalInertia[2], diagonalInertiaUSD[2]);
    checkedDiagInertia = true;
  }
  EXPECT_TRUE(checkedDiagInertia);

  bool checkedCOM = false;
  if (auto comAttr =
      _usdPrim.GetAttribute(pxr::TfToken("physics:centerOfMass")))
  {
    pxr::GfVec3f centerOfMassUSD;
    comAttr.Get(&centerOfMassUSD);
    EXPECT_FLOAT_EQ(_targetCenterOfMass[0], centerOfMassUSD[0]);
    EXPECT_FLOAT_EQ(_targetCenterOfMass[1], centerOfMassUSD[1]);
    EXPECT_FLOAT_EQ(_targetCenterOfMass[2], centerOfMassUSD[2]);
    checkedCOM = true;
  }
  EXPECT_TRUE(checkedCOM);

  EXPECT_EQ(_isRigid, _usdPrim.HasAPI<pxr::UsdPhysicsRigidBodyAPI>());
}
} // namespace testing
} // namespace usd
}
} // namespace sdf

#endif
