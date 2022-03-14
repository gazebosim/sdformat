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

// TODO(ahcorde):this is to remove deprecated "warnings" in usd, these warnings
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
  const ignition::math::Vector3d &_scale)
{
  pxr::UsdPrim prim = _stage->GetPrimAtPath(pxr::SdfPath(_primName));
  ASSERT_TRUE(prim);

  sdf::usd::UDSTransforms usdTransforms =
    sdf::usd::ParseUSDTransform(prim);

  EXPECT_EQ(_translation, usdTransforms.translate);
  EXPECT_EQ(_scale, usdTransforms.scale);
  EXPECT_EQ(_rotation.size(), usdTransforms.q.size());
  for (unsigned int i = 0; i < _rotation.size(); ++i)
  {
    EXPECT_EQ(_rotation[i], usdTransforms.q[i]);
  }

  EXPECT_EQ(!_rotation.empty(), usdTransforms.isRotation);
}

/////////////////////////////////////////////////
TEST(Utils, GetTransform)
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
    ignition::math::Vector3d(1, 1, 1)
  );

  checkTransforms(
    "/shapes/ground_plane/link/visual/geometry",
    stage,
    ignition::math::Vector3d(0, 0, 0),
    {},
    ignition::math::Vector3d(100, 100, 0.25)
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
    ignition::math::Vector3d(1, 1, 1)
  );

  checkTransforms(
    "/shapes/sphere",
    stage,
    ignition::math::Vector3d(0, 1.5, 0.5),
    {
      ignition::math::Quaterniond(1, 0, 0, 0),
      ignition::math::Quaterniond(1, 0, 0, 0),
      ignition::math::Quaterniond(1, 0, 0, 0)
    },
    ignition::math::Vector3d(1, 1, 1)
  );

  checkTransforms(
    "/shapes/capsule",
    stage,
    ignition::math::Vector3d(0, -3.0, 0.5),
    {
      ignition::math::Quaterniond(1, 0, 0, 0),
      ignition::math::Quaterniond(1, 0, 0, 0),
      ignition::math::Quaterniond(1, 0, 0, 0)
    },
    ignition::math::Vector3d(1, 1, 1)
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
    ignition::math::Vector3d(1, 1, 1)
  );

  checkTransforms(
    "/shapes/ellipsoid",
    stage,
    ignition::math::Vector3d(0, 3.0, 0.5),
    {
      ignition::math::Quaterniond(1, 0, 0, 0),
      ignition::math::Quaterniond(1, 0, 0, 0),
      ignition::math::Quaterniond(1, 0, 0, 0)
    },
    ignition::math::Vector3d(1, 1, 1)
  );

  checkTransforms(
    "/shapes/ellipsoid/ellipsoid_link/ellipsoid_visual/geometry",
    stage,
    ignition::math::Vector3d(0, 0, 0),
    {},
    ignition::math::Vector3d(0.4, 0.6, 1)
  );

  checkTransforms(
    "/shapes/sun",
    stage,
    ignition::math::Vector3d(0, 0, 10),
    {
      ignition::math::Quaterniond(1, 0, 0, 0),
      ignition::math::Quaterniond(1, 0, 0, 0),
      ignition::math::Quaterniond(1, 0, 0, 0)
    },
    ignition::math::Vector3d(1, 1, 1)
  );
}

/////////////////////////////////////////////////
TEST(Utils, GetAllTransform)
{
  std::string filename = sdf::testing::TestFile("usd", "upAxisZ.usda");
  auto stage = pxr::UsdStage::Open(filename);
  ASSERT_TRUE(stage);

  sdf::usd::USDData usdData(sdf::testing::TestFile("usd", "upAxisZ.usda"));
  usdData.Init();

  pxr::UsdPrim prim = stage->GetPrimAtPath(
    pxr::SdfPath("/shapes/ellipsoid/ellipsoid_link/ellipsoid_visual/geometry"));
  ASSERT_TRUE(prim);

  ignition::math::Pose3d pose;
  ignition::math::Vector3d scale{1, 1, 1};

  sdf::usd::GetTransform(prim, usdData, pose, scale, "/shapes");

  EXPECT_EQ(ignition::math::Vector3d(0.4, 0.6, 1), scale);
  EXPECT_EQ(
    ignition::math::Pose3d(
      ignition::math::Vector3d(0, 0.03, 0.005),
      ignition::math::Quaterniond(1, 0, 0, 0)),
    pose);
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
