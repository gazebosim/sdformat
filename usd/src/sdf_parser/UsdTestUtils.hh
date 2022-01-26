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

#include <string>
#include <gtest/gtest.h>
#include <ignition/common/Filesystem.hh>
#include <ignition/common/URI.hh>
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
std::string findFileCb(const std::string &_input)
{
  return sdf::testing::TestFile("sdf", _input);
}

//////////////////////////////////////////////////
/// \brief This functions is used by sdf::addFindFileURICallback to find
/// the resources defined in the URI
/// \param[in] _uri URI of the file to find
/// \return The full path to the uri. Empty
/// string is returned if the file could not be found.
std::string FindResourceUri(const ignition::common::URI &_uri)
{
  std::string prefix = _uri.Scheme();
  std::string suffix;
  // Strip /
  if (_uri.Path().IsAbsolute() && prefix != "file")
    suffix += _uri.Path().Str().substr(1);
  else
    suffix += _uri.Path().Str();
  suffix += _uri.Query().Str();

  return findFileCb(ignition::common::copyFromUnixPath(suffix));
}

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
/// \param[in] _mass Mass of the link
/// \param[in] _diagonalInertia Diagonal Inertia
/// \param[in] _centerOfMass Center of mass
/// \param[in] isRigid True if it's a rigid body, False otherwise
void CheckInertial(const pxr::UsdPrim &_usdPrim,
    float _mass,
    const pxr::GfVec3f &_diagonalInertia,
    const pxr::GfVec3f &_centerOfMass,
    bool isRigid)
{
  float massUSD;
  pxr::GfVec3f centerOfMassUSD;
  pxr::GfVec3f diagonalInertiaUSD;

  EXPECT_EQ(isRigid, _usdPrim.HasAPI<pxr::UsdPhysicsRigidBodyAPI>());
  _usdPrim.GetAttribute(pxr::TfToken("physics:mass")).Get(&massUSD);
  EXPECT_FLOAT_EQ(_mass, massUSD);
  _usdPrim.GetAttribute(pxr::TfToken("physics:centerOfMass")).
    Get(&centerOfMassUSD);
  EXPECT_EQ(_centerOfMass, centerOfMassUSD);
  _usdPrim.GetAttribute(pxr::TfToken("physics:diagonalInertia")).
    Get(&diagonalInertiaUSD);
  EXPECT_EQ(_diagonalInertia, diagonalInertiaUSD);
}
} // namespace testing
} // namespace usd
}
} // namespace sdf

#endif
