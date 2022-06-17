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

#include <functional>
#include <optional>
#include <vector>

#include <gtest/gtest.h>

#include <gz/utils/ExtraTestMacros.hh>

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
  const gz::math::Vector3d &_translation,
  const std::optional<gz::math::Quaterniond> &_rotation,
  const gz::math::Vector3d &_scale)
{
  pxr::UsdPrim prim = _stage->GetPrimAtPath(pxr::SdfPath(_primName));
  ASSERT_TRUE(prim);

  sdf::usd::UDSTransforms usdTransforms =
    sdf::usd::ParseUSDTransform(prim);

  EXPECT_EQ(_translation, usdTransforms.Translation());
  EXPECT_EQ(_scale, usdTransforms.Scale());
  if (_rotation)
  {
    ASSERT_TRUE(usdTransforms.Rotation());
    EXPECT_TRUE(gz::math::equal(
      _rotation.value().Roll(),
       usdTransforms.Rotation().value().Roll(), 0.1));
   EXPECT_TRUE(gz::math::equal(
     _rotation.value().Pitch(),
      usdTransforms.Rotation().value().Pitch(), 0.1));
    EXPECT_TRUE(gz::math::equal(
      _rotation.value().Yaw(),
       usdTransforms.Rotation().value().Yaw(), 0.1));
  }
}

/////////////////////////////////////////////////
TEST(USDTransformsTest, ParseUSDTransform)
{
  std::string filename = sdf::testing::TestFile("usd", "upAxisZ.usda");
  auto stage = pxr::UsdStage::Open(filename);
  ASSERT_TRUE(stage);

  checkTransforms(
    "/ground_plane",
    stage,
    gz::math::Vector3d(0, 0, -0.125),
    gz::math::Quaterniond(1, 0, 0, 0),
    gz::math::Vector3d(1, 1, 1)
  );

  checkTransforms(
    "/ground_plane/link/visual/geometry",
    stage,
    gz::math::Vector3d(0, 0, 0),
    std::nullopt,
    gz::math::Vector3d(100, 100, 0.25)
  );

  checkTransforms(
    "/cylinder",
    stage,
    gz::math::Vector3d(0, -1.5, 0.5),
    gz::math::Quaterniond(1, 0, 0, 0),
    gz::math::Vector3d(1, 1, 1)
  );

  checkTransforms(
    "/sphere",
    stage,
    gz::math::Vector3d(0, 1.5, 0.5),
      gz::math::Quaterniond(
        IGN_DTOR(-62), IGN_DTOR(-47.5), IGN_DTOR(-53.41)),
    gz::math::Vector3d(1, 1, 1)
  );

  checkTransforms(
    "/capsule",
    stage,
    gz::math::Vector3d(0, -3.0, 0.5),
    gz::math::Quaterniond(
      IGN_DTOR(-75.1), IGN_DTOR(49.2), IGN_DTOR(-81.2)),
    gz::math::Vector3d(1, 1, 1)
  );

  checkTransforms(
    "/capsule/capsule_link/capsule_visual",
    stage,
    gz::math::Vector3d(0, 0, 0),
    gz::math::Quaterniond(0, 0, M_PI_2),
    gz::math::Vector3d(1, 1, 1)
  );

  checkTransforms(
    "/ellipsoid",
    stage,
    gz::math::Vector3d(0, 3.0, 0.5),
    gz::math::Quaterniond(
      IGN_DTOR(-75.1), IGN_DTOR(49.2), IGN_DTOR(-81.2)),
    gz::math::Vector3d(1, 1, 1)
  );

  checkTransforms(
    "/ellipsoid/ellipsoid_link/ellipsoid_visual/geometry",
    stage,
    gz::math::Vector3d(0, 0, 0),
    std::nullopt,
    gz::math::Vector3d(0.4, 0.6, 1)
  );

  checkTransforms(
    "/sun",
    stage,
    gz::math::Vector3d(0, 0, 10),
      gz::math::Quaterniond(0, IGN_DTOR(-35), 0),
    gz::math::Vector3d(1, 1, 1)
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
        "/ellipsoid/ellipsoid_link/ellipsoid_visual/geometry"));
    ASSERT_TRUE(prim);

    gz::math::Pose3d pose;
    gz::math::Vector3d scale{1, 1, 1};

    sdf::usd::GetTransform(prim, usdData, pose, scale, "/shapes");

    EXPECT_EQ(gz::math::Vector3d(0.4, 0.6, 1), scale);
    EXPECT_EQ(
      gz::math::Pose3d(
        gz::math::Vector3d(0, 0.03, 0.005),
        gz::math::Quaterniond(
          IGN_DTOR(-75.1), IGN_DTOR(49.2), IGN_DTOR(-81.2))),
      pose);
  }

  {
    std::string filename =
      sdf::testing::TestFile("usd", "nested_transforms.usda");
    auto stage = pxr::UsdStage::Open(filename);
    ASSERT_TRUE(stage);

    sdf::usd::USDData usdData(filename);
    usdData.Init();

    std::function<void(
      const std::string &,
      const gz::math::Vector3d &,
      const gz::math::Quaterniond &)> verifyNestedTf =
      [&](const std::string &_path,
          const gz::math::Vector3d &_posePrim,
          const gz::math::Quaterniond &_qPrim)
      {
        pxr::UsdPrim prim = stage->GetPrimAtPath(pxr::SdfPath(_path));
        ASSERT_TRUE(prim);

        gz::math::Pose3d pose;
        gz::math::Vector3d scale{1, 1, 1};

        sdf::usd::GetTransform(prim, usdData, pose, scale, "/transforms");

        EXPECT_EQ(gz::math::Vector3d(1, 1, 1), scale);
        EXPECT_EQ(gz::math::Pose3d(_posePrim, _qPrim), pose);
      };

    verifyNestedTf(
      "/transforms/nested_transforms_XYZ/child_transform",
        gz::math::Vector3d(0.01, 0.01, 0),
        gz::math::Quaterniond(0, 0, IGN_DTOR(90)));
    verifyNestedTf(
      "/transforms/nested_transforms_ZYX/child_transform",
      gz::math::Vector3d(0.02, 0.0, 0),
      gz::math::Quaterniond(IGN_DTOR(90), 0, 0));
  }
}
