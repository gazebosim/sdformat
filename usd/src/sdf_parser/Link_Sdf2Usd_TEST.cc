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

// TODO(ahcorde) this is to remove deprecated "warnings" in usd, these warnings
// are reported using #pragma message so normal diagnostic flags cannot remove
// them. This workaround requires this block to be used whenever usd is
// included.
#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/base/gf/quatf.h>
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/tf/token.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/tokens.h>
#include <pxr/usd/usdPhysics/scene.h>
#pragma pop_macro ("__DEPRECATED")

#include <ignition/common/Util.hh>

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
TEST_F(UsdStageFixture, Link)
{
  sdf::setFindCallback(sdf::usd::testing::findFileCb);
  ignition::common::addFindFileURICallback(
    std::bind(&sdf::usd::testing::FindResourceUri, std::placeholders::_1));

  const auto path = sdf::testing::TestFile("sdf", "basic_shapes.sdf");
  sdf::Root root;

  ASSERT_TRUE(sdf::testing::LoadSdfFile(path, root));
  ASSERT_EQ(1u, root.WorldCount());
  auto world = root.WorldByIndex(0u);

  const auto worldPath = std::string("/" + world->Name());
  auto usdErrors = sdf::usd::ParseSdfWorld(*world, stage, worldPath);
  EXPECT_TRUE(usdErrors.empty());

  auto worldPrim = this->stage->GetPrimAtPath(pxr::SdfPath(worldPath));
  ASSERT_TRUE(worldPrim);

  std::string groundPlanePath = worldPath + "/" + "ground_plane";
  auto groundPlane = this->stage->GetPrimAtPath(pxr::SdfPath(groundPlanePath));
  ASSERT_TRUE(groundPlane);
  sdf::usd::testing::CheckPrimPose(groundPlane,
      ignition::math::Pose3d(
        ignition::math::Vector3d(0, 0, -0.125),
        ignition::math::Quaterniond(0, 0, 0)));
  std::string groundPlaneLinkPath = groundPlanePath + "/" + "link";
  auto groundPlaneLink = this->stage->GetPrimAtPath(
    pxr::SdfPath(groundPlaneLinkPath));
  ASSERT_TRUE(groundPlaneLink);
  sdf::usd::testing::CheckPrimPose(groundPlaneLink,
      ignition::math::Pose3d(
        ignition::math::Vector3d(0, 0, 0),
        ignition::math::Quaterniond(0, 0, 0)));

  std::string boxPath = worldPath + "/" + "box";
  auto box = this->stage->GetPrimAtPath(pxr::SdfPath(boxPath));
  ASSERT_TRUE(box);
  sdf::usd::testing::CheckPrimPose(box,
      ignition::math::Pose3d(
        ignition::math::Vector3d(0, 0, 2.5),
        ignition::math::Quaterniond(0, 0, 0)));
  std::string boxLinkPath = boxPath + "/" + "link";
  auto boxLink = this->stage->GetPrimAtPath(pxr::SdfPath(boxLinkPath));
  ASSERT_TRUE(boxLink);
  sdf::usd::testing::CheckInertial(
    boxLink, 1, pxr::GfVec3f(1, 1, 1), pxr::GfQuatf(1, 0, 0, 0),
    pxr::GfVec3f(0, 0, 0), true);
  sdf::usd::testing::CheckPrimPose(boxLink,
      ignition::math::Pose3d(
        ignition::math::Vector3d(0, 0, 0),
        ignition::math::Quaterniond(0, 0, 0)));

  std::string cylinderPath = worldPath + "/" + "cylinder";
  auto cylinder = this->stage->GetPrimAtPath(pxr::SdfPath(cylinderPath));
  ASSERT_TRUE(cylinder);
  sdf::usd::testing::CheckPrimPose(cylinder,
      ignition::math::Pose3d(
        ignition::math::Vector3d(2, 0, 2.5),
        ignition::math::Quaterniond(0, 0, 0)));
  std::string cylinderLinkPath = cylinderPath + "/" + "link";
  auto cylinderLink =
      this->stage->GetPrimAtPath(pxr::SdfPath(cylinderLinkPath));
  ASSERT_TRUE(cylinderLink);
  sdf::usd::testing::CheckInertial(
    cylinderLink, 1, pxr::GfVec3f(1, 1, 1), pxr::GfQuatf(1, 0, 0, 0),
    pxr::GfVec3f(0, 0, 0), true);
  sdf::usd::testing::CheckPrimPose(cylinderLink,
      ignition::math::Pose3d(
        ignition::math::Vector3d(0, 0, 0),
        ignition::math::Quaterniond(0, 0, 0)));

  std::string spherePath = worldPath + "/" + "sphere";
  auto sphere = this->stage->GetPrimAtPath(pxr::SdfPath(spherePath));
  ASSERT_TRUE(sphere);
  sdf::usd::testing::CheckPrimPose(sphere,
      ignition::math::Pose3d(
        ignition::math::Vector3d(4, 0, 2.5),
        ignition::math::Quaterniond(0, 0, 0)));
  std::string sphereLinkPath = spherePath + "/" + "link";
  auto sphereLink = this->stage->GetPrimAtPath(pxr::SdfPath(sphereLinkPath));
  ASSERT_TRUE(sphereLink);
  sdf::usd::testing::CheckInertial(
    sphereLink, 1, pxr::GfVec3f(1, 1, 1), pxr::GfQuatf(1, 0, 0, 0),
    pxr::GfVec3f(0, 0, 0), true);
  sdf::usd::testing::CheckPrimPose(sphereLink,
      ignition::math::Pose3d(
        ignition::math::Vector3d(0, 0, 0),
        ignition::math::Quaterniond(0, 0, 0)));

  std::string capsulePath = worldPath + "/" + "capsule";
  auto capsule = this->stage->GetPrimAtPath(pxr::SdfPath(capsulePath));
  ASSERT_TRUE(capsule);
  sdf::usd::testing::CheckPrimPose(capsule,
      ignition::math::Pose3d(
        ignition::math::Vector3d(6, 0, 2.5),
        ignition::math::Quaterniond(0, 0, 0)));
  std::string capsuleLinkPath = capsulePath + "/" + "link";
  auto capsuleLink = this->stage->GetPrimAtPath(pxr::SdfPath(capsuleLinkPath));
  ASSERT_TRUE(capsuleLink);
  sdf::usd::testing::CheckInertial(
    capsuleLink, 1, pxr::GfVec3f(1, 1, 1), pxr::GfQuatf(1, 0, 0, 0),
    pxr::GfVec3f(0, 0, 0), true);
  sdf::usd::testing::CheckPrimPose(capsuleLink,
      ignition::math::Pose3d(
        ignition::math::Vector3d(0, 0, 0),
        ignition::math::Quaterniond(0, 0, 0)));

  std::string meshPath = worldPath + "/" + "mesh";
  auto mesh = this->stage->GetPrimAtPath(pxr::SdfPath(meshPath));
  ASSERT_TRUE(mesh);
  sdf::usd::testing::CheckPrimPose(mesh,
      ignition::math::Pose3d(
        ignition::math::Vector3d(8, 0, 2.5),
        ignition::math::Quaterniond(0, 0, 0)));
  std::string meshLinkPath = meshPath + "/" + "link";
  auto meshLink = this->stage->GetPrimAtPath(pxr::SdfPath(meshLinkPath));
  ASSERT_TRUE(meshLink);
  sdf::usd::testing::CheckInertial(
    meshLink, 1, pxr::GfVec3f(1, 1, 1), pxr::GfQuatf(1, 0, 0, 0),
    pxr::GfVec3f(0, 0, 0), true);
  sdf::usd::testing::CheckPrimPose(meshLink,
      ignition::math::Pose3d(
        ignition::math::Vector3d(0, 0, 0),
        ignition::math::Quaterniond(0, 0, 0)));
}
