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
#ifndef SDF_USD_TEST_UTILS_HH_
#define SDF_USD_TEST_UTILS_HH_

#include <gtest/gtest.h>
#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>
#include <pxr/base/gf/vec3d.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/tf/token.h>
#include <pxr/usd/usd/prim.h>

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

} // namespace testing
} // namespace usd

#endif
