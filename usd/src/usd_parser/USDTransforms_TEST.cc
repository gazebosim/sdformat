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

#include <vector>

#include <gtest/gtest.h>

#include <ignition/utilities/ExtraTestMacros.hh>

#include "test_config.h"
#include "test_utils.hh"

// TODO(ahcorde) this is to remove deprecated "warnings" in usd, these warnings
// are reported using #pragma message so normal diagnostic flags cannot remove
// them. This workaround requires this block to be used whenever usd is
// included.
#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/usd/usd/stage.h>
#pragma pop_macro ("__DEPRECATED")

#include "sdf/usd/usd_parser/USDTransforms.hh"

void checkTransforms(
  const std::string &_primName,
  pxr::UsdStageRefPtr &_stage,
  const ignition::math::Vector3d &_translation,
  const std::vector<ignition::math::Quaterniond> &_rotation,
  const ignition::math::Vector3d &_scale,
  bool isXYZRotation,
  bool isZYXRotation)
{
  pxr::UsdPrim prim = _stage->GetPrimAtPath(pxr::SdfPath(_primName));
  ASSERT_TRUE(prim);

  sdf::usd::UDSTransforms usdTransforms =
    sdf::usd::ParseUSDTransform(prim);

  EXPECT_EQ(_translation, usdTransforms.Translation());
  EXPECT_EQ(_scale, usdTransforms.Scale());
  ASSERT_EQ(_rotation.size(), usdTransforms.Rotations().size());
  for (unsigned int i = 0; i < _rotation.size(); ++i)
  {
    EXPECT_EQ(_rotation[i], usdTransforms.Rotations()[i]);
  }

  EXPECT_EQ(!_rotation.empty(), usdTransforms.Rotation());
  EXPECT_EQ(isXYZRotation, usdTransforms.RotationXYZ());
  EXPECT_EQ(isZYXRotation, usdTransforms.RotationZYX());
  const bool hasRotation = isXYZRotation || isZYXRotation;
  if (hasRotation)
  {
    EXPECT_NE(usdTransforms.RotationXYZ(), usdTransforms.RotationZYX());
  }
  EXPECT_EQ(hasRotation, usdTransforms.Rotation());
}

/////////////////////////////////////////////////
TEST(USDTransformsTest, ParseUSDTransform)
{
  std::string filename = sdf::testing::TestFile("usd", "upAxisZ.usda");
  auto stage = pxr::UsdStage::Open(filename);
  ASSERT_TRUE(stage);

  checkTransforms(
    "/shapes/ground_plane",
    stage,
    ignition::math::Vector3d(0, 0, -0.125),
    {
      ignition::math::Quaterniond(1, 0, 0, 0),
      ignition::math::Quaterniond(1, 0, 0, 0),
      ignition::math::Quaterniond(1, 0, 0, 0)
    },
    ignition::math::Vector3d(1, 1, 1),
    true,
    false
  );

  checkTransforms(
    "/shapes/ground_plane/link/visual/geometry",
    stage,
    ignition::math::Vector3d(0, 0, 0),
    {},
    ignition::math::Vector3d(100, 100, 0.25),
    false,
    false
  );

  checkTransforms(
    "/shapes/cylinder",
    stage,
    ignition::math::Vector3d(0, -1.5, 0.5),
    {
      ignition::math::Quaterniond(1, 0, 0, 0),
      ignition::math::Quaterniond(1, 0, 0, 0),
      ignition::math::Quaterniond(1, 0, 0, 0)
    },
    ignition::math::Vector3d(1, 1, 1),
    true,
    false
  );

  checkTransforms(
    "/shapes/sphere",
    stage,
    ignition::math::Vector3d(0, 1.5, 0.5),
    {
      ignition::math::Quaterniond(IGN_DTOR(-69), 0, 0),
      ignition::math::Quaterniond(0, IGN_DTOR(31), 0),
      ignition::math::Quaterniond(0, 0, IGN_DTOR(-62))
    },
    ignition::math::Vector3d(1, 1, 1),
    false,
    true
  );

  checkTransforms(
    "/shapes/capsule",
    stage,
    ignition::math::Vector3d(0, -3.0, 0.5),
    {
      ignition::math::Quaterniond(IGN_DTOR(15), 0, 0),
      ignition::math::Quaterniond(0, IGN_DTOR(80), 0),
      ignition::math::Quaterniond(0, 0, IGN_DTOR(-55))
    },
    ignition::math::Vector3d(1, 1, 1),
    false,
    true
  );

  checkTransforms(
    "/shapes/capsule/capsule_link/capsule_visual",
    stage,
    ignition::math::Vector3d(0, 0, 0),
    {
      ignition::math::Quaterniond(1, 0, 0, 0),
      ignition::math::Quaterniond(1, 0, 0, 0),
      ignition::math::Quaterniond(0, 0, M_PI_2)
    },
    ignition::math::Vector3d(1, 1, 1),
    false,
    true
  );

  checkTransforms(
    "/shapes/ellipsoid",
    stage,
    ignition::math::Vector3d(0, 3.0, 0.5),
    {
      ignition::math::Quaterniond(IGN_DTOR(15), 0, 0),
      ignition::math::Quaterniond(0, IGN_DTOR(80), 0),
      ignition::math::Quaterniond(0, 0, IGN_DTOR(-55))
    },
    ignition::math::Vector3d(1, 1, 1),
    true,
    false
  );

  checkTransforms(
    "/shapes/ellipsoid/ellipsoid_link/ellipsoid_visual/geometry",
    stage,
    ignition::math::Vector3d(0, 0, 0),
    {},
    ignition::math::Vector3d(0.4, 0.6, 1),
    false,
    false
  );

  checkTransforms(
    "/shapes/sun",
    stage,
    ignition::math::Vector3d(0, 0, 10),
    {
      ignition::math::Quaterniond(1, 0, 0, 0),
      ignition::math::Quaterniond(0, IGN_DTOR(-35), 0),
      ignition::math::Quaterniond(1, 0, 0, 0)
    },
    ignition::math::Vector3d(1, 1, 1),
    true,
    false
  );
}

/////////////////////////////////////////////////
TEST(USDTransformsTest, GetAllTransform)
{
  {
    std::string filename = sdf::testing::TestFile("usd", "upAxisZ.usda");
    auto stage = pxr::UsdStage::Open(filename);
    ASSERT_TRUE(stage);

    sdf::usd::USDData usdData(filename);
    usdData.Init();

    pxr::UsdPrim prim = stage->GetPrimAtPath(
      pxr::SdfPath(
        "/shapes/ellipsoid/ellipsoid_link/ellipsoid_visual/geometry"));
    ASSERT_TRUE(prim);

    ignition::math::Pose3d pose;
    ignition::math::Vector3d scale{1, 1, 1};

    sdf::usd::GetTransform(prim, usdData, pose, scale, "/shapes");

    EXPECT_EQ(ignition::math::Vector3d(0.4, 0.6, 1), scale);
    EXPECT_EQ(
      ignition::math::Pose3d(
        ignition::math::Vector3d(0, 0.03, 0.005),
        ignition::math::Quaterniond(IGN_DTOR(15), IGN_DTOR(80), IGN_DTOR(-55))),
      pose);
  }

  {
    std::string filename =
      sdf::testing::TestFile("usd", "nested_transforms.usda");
    auto stage = pxr::UsdStage::Open(filename);
    ASSERT_TRUE(stage);

    sdf::usd::USDData usdData(filename);
    usdData.Init();

    const pxr::UsdPrim prim = stage->GetPrimAtPath(
        pxr::SdfPath("/transforms/nested_transforms_XYZ/child_transform"));
    ASSERT_TRUE(prim);

    ignition::math::Pose3d pose;
    ignition::math::Vector3d scale{1, 1, 1};

    sdf::usd::GetTransform(prim, usdData, pose, scale, "/transforms");

    EXPECT_EQ(ignition::math::Vector3d(1, 1, 1), scale);
    EXPECT_EQ(
      ignition::math::Pose3d(
        ignition::math::Vector3d(.01, .01, 0),
        ignition::math::Quaterniond(0, 0, IGN_DTOR(90))),
      pose);
  }
}
