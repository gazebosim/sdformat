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
#include <pxr/base/gf/vec3f.h>
#include <pxr/base/tf/token.h>
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/primRange.h>
#include <pxr/usd/usd/stage.h>
#include <pxr/usd/usdGeom/tokens.h>
#include <pxr/usd/usdPhysics/scene.h>
#pragma pop_macro ("__DEPRECATED")

#include "sdf/usd/sdf_parser/World.hh"
#include "sdf/Root.hh"
#include "test_config.h"
#include "test_utils.hh"

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
TEST_F(UsdStageFixture, World)
{
  const auto path = sdf::testing::TestFile("sdf", "empty.sdf");
  sdf::Root root;

  ASSERT_TRUE(sdf::testing::LoadSdfFile(path, root));
  ASSERT_EQ(1u, root.WorldCount());
  auto world = root.WorldByIndex(0u);

  const auto worldPath = std::string("/" + world->Name());
  auto usdErrors = sdf::usd::ParseSdfWorld(*world, stage, worldPath);
  EXPECT_TRUE(usdErrors.empty());

  // check top-level stage information
  EXPECT_DOUBLE_EQ(100.0, this->stage->GetEndTimeCode());
  EXPECT_DOUBLE_EQ(0.0, this->stage->GetStartTimeCode());
  EXPECT_DOUBLE_EQ(24.0, this->stage->GetTimeCodesPerSecond());
  pxr::TfToken upAxisVal;
  EXPECT_TRUE(this->stage->GetMetadata(pxr::UsdGeomTokens->upAxis, &upAxisVal));
  EXPECT_EQ(pxr::UsdGeomTokens->z, upAxisVal);
  double metersPerUnitVal;
  EXPECT_TRUE(this->stage->GetMetadata(pxr::TfToken("metersPerUnit"),
        &metersPerUnitVal));
  EXPECT_DOUBLE_EQ(1.0, metersPerUnitVal);

  // Check that world prim exists, and that things like physics information
  // were parsed correctly
  auto worldPrim = this->stage->GetPrimAtPath(pxr::SdfPath(worldPath));
  ASSERT_TRUE(worldPrim);
  auto physicsScene = pxr::UsdPhysicsScene::Get(this->stage,
      pxr::SdfPath(worldPath + "/physics"));
  ASSERT_TRUE(physicsScene);
  pxr::GfVec3f gravityDirectionVal;
  EXPECT_TRUE(physicsScene.GetGravityDirectionAttr().Get(&gravityDirectionVal));
  EXPECT_EQ(gravityDirectionVal, pxr::GfVec3f(0, 0, -1));
  float gravityMagnitudeVal;
  EXPECT_TRUE(physicsScene.GetGravityMagnitudeAttr().Get(&gravityMagnitudeVal));
  EXPECT_FLOAT_EQ(gravityMagnitudeVal, 9.8f);
}
