/*
 * Copyright 2022 Open Source Robotics Foundation
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

#include <string>

#include <gtest/gtest.h>

#include <ignition/common/Util.hh>

// TODO(ahcorde) this is to remove deprecated "warnings" in usd, these warnings
// are reported using #pragma message so normal diagnostic flags cannot remove
// them. This workaround requires this block to be used whenever usd is
// included.
#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/base/gf/vec3f.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/stage.h>
#pragma pop_macro ("__DEPRECATED")

#include "sdf/usd/sdf_parser/World.hh"
#include "sdf/Root.hh"
#include "test_config.h"
#include "test_utils.hh"
#include "../UsdTestUtils.hh"

/////////////////////////////////////////////////
// Fixture that creates a USD stage for each test case.
class UsdStageFixture : public::testing::Test
{
  public: UsdStageFixture() = default;

  protected: void SetUp() override
  {
    this->stage = pxr::UsdStage::CreateInMemory();
    ASSERT_TRUE(this->stage);
  }

  public: pxr::UsdStageRefPtr stage;
};

/////////////////////////////////////////////////
TEST_F(UsdStageFixture, Visual)
{
  sdf::setFindCallback(sdf::usd::testing::findFileCb);
  gz::common::addFindFileURICallback(
    std::bind(&sdf::usd::testing::FindResourceUri, std::placeholders::_1));

  const auto path = sdf::testing::TestFile("sdf", "basic_shapes.sdf");
  sdf::Root root;

  ASSERT_TRUE(sdf::testing::LoadSdfFile(path, root));
  ASSERT_EQ(1u, root.WorldCount());
  const auto world = root.WorldByIndex(0u);

  const auto worldPath = std::string("/" + world->Name());
  const auto usdErrors = sdf::usd::ParseSdfWorld(*world, stage, worldPath);
  EXPECT_TRUE(usdErrors.empty());

  const auto worldPrim = this->stage->GetPrimAtPath(pxr::SdfPath(worldPath));
  ASSERT_TRUE(worldPrim);

  const auto targetPose = gz::math::Pose3d(
      gz::math::Vector3d(0, 0, 0),
      gz::math::Quaterniond(0, 0, 0));

  const std::string groundPlanePath = worldPath + "/ground_plane";
  const auto groundPlane =
    this->stage->GetPrimAtPath(pxr::SdfPath(groundPlanePath));
  ASSERT_TRUE(groundPlane);
  const std::string groundPlaneLinkPath = groundPlanePath + "/link";
  const auto groundPlaneLink = this->stage->GetPrimAtPath(
    pxr::SdfPath(groundPlaneLinkPath));
  ASSERT_TRUE(groundPlaneLink);
  const std::string groundPlaneVisualPath = groundPlaneLinkPath + "/visual";
  const auto groundPlaneVisual = this->stage->GetPrimAtPath(
    pxr::SdfPath(groundPlaneVisualPath));
  ASSERT_TRUE(groundPlaneVisual);
  sdf::usd::testing::CheckPrimPose(groundPlaneVisual, targetPose);

  const std::string boxPath = worldPath + "/box";
  const auto box = this->stage->GetPrimAtPath(pxr::SdfPath(boxPath));
  ASSERT_TRUE(box);
  const std::string boxLinkPath = boxPath + "/link";
  const auto boxLink = this->stage->GetPrimAtPath(pxr::SdfPath(boxLinkPath));
  ASSERT_TRUE(boxLink);
  const std::string boxVisualPath = boxLinkPath + "/box_vis";
  const auto boxVisual = this->stage->GetPrimAtPath(
    pxr::SdfPath(boxVisualPath));
  ASSERT_TRUE(boxVisual);
  sdf::usd::testing::CheckPrimPose(boxVisual, targetPose);

  const std::string cylinderPath = worldPath + "/cylinder";
  auto cylinder = this->stage->GetPrimAtPath(pxr::SdfPath(cylinderPath));
  ASSERT_TRUE(cylinder);
  std::string cylinderLinkPath = cylinderPath + "/link";
  const auto cylinderLink =
    this->stage->GetPrimAtPath(pxr::SdfPath(cylinderLinkPath));
  ASSERT_TRUE(cylinderLink);
  const std::string cylinderVisualPath = cylinderLinkPath + "/visual";
  const auto cylinderVisual = this->stage->GetPrimAtPath(
    pxr::SdfPath(cylinderVisualPath));
  ASSERT_TRUE(cylinderVisual);
  sdf::usd::testing::CheckPrimPose(cylinderVisual, targetPose);

  const std::string spherePath = worldPath + "/sphere";
  const auto sphere = this->stage->GetPrimAtPath(pxr::SdfPath(spherePath));
  ASSERT_TRUE(sphere);
  const std::string sphereLinkPath = spherePath + "/link";
  const auto sphereLink =
    this->stage->GetPrimAtPath(pxr::SdfPath(sphereLinkPath));
  ASSERT_TRUE(sphereLink);
  const std::string sphereVisualPath = sphereLinkPath + "/sphere_vis";
  const auto sphereVisual = this->stage->GetPrimAtPath(
    pxr::SdfPath(sphereVisualPath));
  ASSERT_TRUE(sphereVisual);
  sdf::usd::testing::CheckPrimPose(sphereVisual, targetPose);

  const std::string capsulePath = worldPath + "/capsule";
  const auto capsule = this->stage->GetPrimAtPath(pxr::SdfPath(capsulePath));
  ASSERT_TRUE(capsule);
  const std::string capsuleLinkPath = capsulePath + "/link";
  const auto capsuleLink =
    this->stage->GetPrimAtPath(pxr::SdfPath(capsuleLinkPath));
  ASSERT_TRUE(capsuleLink);
  const std::string capsuleVisualPath = capsuleLinkPath + "/visual";
  const auto capsuleVisual = this->stage->GetPrimAtPath(
    pxr::SdfPath(capsuleVisualPath));
  ASSERT_TRUE(capsuleVisual);
  sdf::usd::testing::CheckPrimPose(capsuleVisual, targetPose);

  const std::string meshPath = worldPath + "/mesh";
  const auto mesh = this->stage->GetPrimAtPath(pxr::SdfPath(meshPath));
  ASSERT_TRUE(mesh);
  const std::string meshLinkPath = meshPath + "/link";
  const auto meshLink = this->stage->GetPrimAtPath(pxr::SdfPath(meshLinkPath));
  ASSERT_TRUE(meshLink);
  const std::string meshVisualPath = meshLinkPath + "/visual";
  const auto meshVisual = this->stage->GetPrimAtPath(
    pxr::SdfPath(meshVisualPath));
  ASSERT_TRUE(meshVisual);
  sdf::usd::testing::CheckPrimPose(meshVisual, targetPose);
}
