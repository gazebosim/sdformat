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

// TODO(ahcorde) This is to remove deprecated "warnings" in usd, these warnings
// are reported using #pragma message so normal diagnostic flags cannot remove
// them. This workaround requires this block to be used whenever usd is
// included.
#pragma push_macro ("__DEPRECATED")
#undef __DEPRECATED
#include <pxr/usd/usd/prim.h>
#include <pxr/usd/usd/stage.h>
#pragma pop_macro ("__DEPRECATED")

#include <sdf/usd/usd_parser/USDData.hh>
#include <sdf/usd/UsdError.hh>

#include <ignition/math/Color.hh>
#include <ignition/math/Pose3.hh>

#include "test_config.h"
#include "test_utils.hh"

#include "sdf/Light.hh"

#include "USDLights.hh"

/////////////////////////////////////////////////
TEST(USDLightsTest, DistanceLight)
{
  std::string filename = sdf::testing::TestFile("usd", "upAxisZ.usda");
  sdf::usd::USDData usdData(filename);
  EXPECT_EQ(0u, usdData.Init().size());

  auto stage = pxr::UsdStage::Open(filename);
  ASSERT_TRUE(stage);

  pxr::UsdPrim prim = stage->GetPrimAtPath(
    pxr::SdfPath("/defaultLight"));
  ASSERT_TRUE(prim);

  auto light = sdf::usd::ParseUSDLights(
    prim, usdData, "");
  ASSERT_TRUE(light);

  EXPECT_EQ("defaultLight", light->Name());
  EXPECT_EQ(sdf::LightType::DIRECTIONAL, light->Type());
  EXPECT_TRUE(light->CastShadows());
  EXPECT_EQ(gz::math::Color(0.5, 0.5, 0.5, 1), light->Diffuse());
  EXPECT_EQ(gz::math::Color(1, 1, 1, 1), light->Specular());
  EXPECT_FLOAT_EQ(0.5, light->Intensity());
  EXPECT_EQ(gz::math::Pose3d(0, 0, 10, 0, IGN_DTOR(45), 0),
      light->RawPose());

  prim = stage->GetPrimAtPath(pxr::SdfPath("/diskLight"));
  ASSERT_TRUE(prim);

  auto diskLight = sdf::usd::ParseUSDLights(
    prim, usdData, "");
  ASSERT_TRUE(diskLight);

  EXPECT_EQ("diskLight", diskLight->Name());
  EXPECT_EQ(sdf::LightType::SPOT, diskLight->Type());
  EXPECT_TRUE(diskLight->CastShadows());
  EXPECT_EQ(gz::math::Color(1, 1, 1, 1), diskLight->Diffuse());
  EXPECT_EQ(gz::math::Color(0.5, 0.5, 0.5, 1), diskLight->Specular());
  EXPECT_FLOAT_EQ(0.3, diskLight->Intensity());
  EXPECT_EQ(
    gz::math::Pose3d(0, 0, 10, 0, 0, IGN_DTOR(45)), diskLight->RawPose());
  EXPECT_DOUBLE_EQ(0.1, diskLight->SpotInnerAngle().Radian());
  EXPECT_DOUBLE_EQ(0.5, diskLight->SpotOuterAngle().Radian());
  EXPECT_DOUBLE_EQ(0.8, diskLight->SpotFalloff());
}
