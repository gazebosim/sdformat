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

#include <gtest/gtest.h>

#include <gz/math/Pose3.hh>

// TODO(adlarkin) this is to remove deprecated "warnings" in usd, these warnings
// are reported using #pragma message so normal diagnostic flags cannot remove
// them. This workaround requires this block to be used whenever usd is
// included.
#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/usd/sdf/path.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/xform.h>
#pragma pop_macro ("__DEPRECATED")

#include "sdf/Light.hh"
#include "sdf/Link.hh"
#include "sdf/Model.hh"
#include "sdf/Root.hh"
#include "sdf/World.hh"
#include "test_config.hh"
#include "test_utils.hh"
#include "UsdTestUtils.hh"
#include "UsdUtils.hh"

//////////////////////////////////////////////////
TEST(UsdUtils, PoseWrtParent)
{
  const auto path = sdf::testing::TestFile("sdf", "model_link_relative_to.sdf");
  sdf::Root root;

  ASSERT_TRUE(sdf::testing::LoadSdfFile(path, root));
  const auto model = root.Model();
  ASSERT_NE(nullptr, model);

  const auto linkL1 = model->LinkByName("L1");
  ASSERT_NE(nullptr, linkL1);
  const auto linkL2 = model->LinkByName("L2");
  ASSERT_NE(nullptr, linkL2);
  const auto linkL3 = model->LinkByName("L3");
  ASSERT_NE(nullptr, linkL3);

  gz::math::Pose3d pose;
  auto errors = sdf::usd::PoseWrtParent(*linkL2, pose);
  EXPECT_TRUE(errors.empty());
  EXPECT_EQ(gz::math::Pose3d(2, 0, 0, 0, 0, 0), pose);

  errors = sdf::usd::PoseWrtParent(*linkL3, pose);
  EXPECT_TRUE(errors.empty());
  EXPECT_EQ(gz::math::Pose3d(1, 0, -3, 0, linkL1->RawPose().Pitch(), 0),
      pose);
}

//////////////////////////////////////////////////
TEST(UsdUtils, SetPose)
{
  auto stage = pxr::UsdStage::CreateInMemory();
  ASSERT_TRUE(stage);

  const pxr::SdfPath primPath("/prim");

  // We create a UsdGeomXform because this is USD's concrete prim schema for
  // transforms
  ASSERT_TRUE(pxr::UsdGeomXform::Define(stage, primPath));

  auto prim = stage->GetPrimAtPath(primPath);
  ASSERT_TRUE(prim);

  const gz::math::Pose3d pose(1, 2, 3, 0, 0, 0);
  auto errors = sdf::usd::SetPose(pose, stage, primPath);
  EXPECT_TRUE(errors.empty());

  sdf::usd::testing::CheckPrimPose(prim, pose);
}

//////////////////////////////////////////////////
TEST(UsdUtils, IsPlane)
{
  const auto path = sdf::testing::TestFile("sdf", "empty.sdf");
  sdf::Root root;

  ASSERT_TRUE(sdf::testing::LoadSdfFile(path, root));
  ASSERT_EQ(1u, root.WorldCount());
  const auto world = root.WorldByIndex(0u);
  ASSERT_NE(nullptr, world);

  ASSERT_EQ(1u, world->ModelCount());
  const auto model = world->ModelByIndex(0u);
  ASSERT_NE(nullptr, model);
  EXPECT_TRUE(sdf::usd::IsPlane(*model));

  // make the model non-static to verify it's no longer considered a plane
  auto mutableModel = const_cast<sdf::Model *>(model);
  ASSERT_NE(nullptr, mutableModel);
  mutableModel->SetStatic(false);
  EXPECT_FALSE(sdf::usd::IsPlane(*mutableModel));
}

//////////////////////////////////////////////////
TEST(UsdUtils, validPath)
{
  const std::string alreadyValid = "/valid/path";
  EXPECT_EQ(alreadyValid, sdf::usd::validPath(alreadyValid));

  EXPECT_EQ("", sdf::usd::validPath(""));

  EXPECT_EQ("_0/start/with/digit", sdf::usd::validPath("0/start/with/digit"));

  EXPECT_EQ("/hasSpaces", sdf::usd::validPath("/has Spaces"));

  EXPECT_EQ("/has_period", sdf::usd::validPath("/has.period"));

  EXPECT_EQ("/has_dash", sdf::usd::validPath("/has-dash"));

  EXPECT_EQ("_5/has_period/hasSpace/has_dash",
      sdf::usd::validPath("5/has.period/has Space/has-dash"));
}
