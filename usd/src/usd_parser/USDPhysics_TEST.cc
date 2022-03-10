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

#include <memory>
#include <string>

// TODO(ahcorde):this is to remove deprecated "warnings" in usd, these warnings
// are reported using #pragma message so normal diagnostic flags cannot remove
// them. This workaround requires this block to be used whenever usd is
// included.
#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/stage.h>
#pragma pop_macro ("__DEPRECATED")

#include "test_config.h"
#include "test_utils.hh"

#include "USDPhysics.hh"
#include "usd_model/WorldInterface.hh"

/////////////////////////////////////////////////
TEST(USDPhysicsTest, AvailablePhysics)
{
  std::string filename = sdf::testing::TestFile("usd", "upAxisZ.usda");
  auto stage = pxr::UsdStage::Open(filename);
  ASSERT_TRUE(stage);

  pxr::UsdPrim prim = stage->GetPrimAtPath(pxr::SdfPath("/shapes/physics"));

  std::shared_ptr<sdf::usd::WorldInterface> worldInterface =
    std::make_shared<sdf::usd::WorldInterface>();

  double metersPerUnit = 1.0;

  sdf::usd::ParseUSDPhysicsScene(
    prim, worldInterface, metersPerUnit);
  EXPECT_EQ(ignition::math::Vector3d(0, 0, -1), worldInterface->gravity);
  EXPECT_NEAR(9.8, worldInterface->magnitude, 0.0001);
}

/////////////////////////////////////////////////
TEST(USDPhysicsTest, UnavailablePhysics)
{
  std::string filename = sdf::testing::TestFile("usd", "upAxisY.usda");
  auto stage = pxr::UsdStage::Open(filename);
  ASSERT_TRUE(stage);

  std::shared_ptr<sdf::usd::WorldInterface> worldInterface =
    std::make_shared<sdf::usd::WorldInterface>();

  EXPECT_EQ(ignition::math::Vector3d(0, 0, -1), worldInterface->gravity);
  EXPECT_NEAR(9.8, worldInterface->magnitude, 0.0001);
}
