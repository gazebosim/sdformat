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

#include <ignition/common/Filesystem.hh>
#include <ignition/common/Util.hh>

#include <ignition/utilities/ExtraTestMacros.hh>

#include <sdf/usd/usd_parser/USDData.hh>
#include <sdf/usd/UsdError.hh>

#include "test_config.h"
#include "test_utils.hh"

#include <sdf/Material.hh>
#include <sdf/Pbr.hh>

/////////////////////////////////////////////////
TEST(USDData, Constructor)
{
  // Open a invalid USD file
  sdf::usd::USDData data(sdf::testing::TestFile("usd", "invalid_name"));
  sdf::usd::UsdErrors errors = data.Init();
  EXPECT_EQ(1u, errors.size());

  // Add test/usd directory to find some resources
  auto systemPaths = ignition::common::systemPaths();
  systemPaths->AddFilePaths(sdf::testing::TestFile("usd"));

  // Open a valid USD file
  {
    std::string filename = sdf::testing::TestFile("usd", "upAxisZ.usda");
    sdf::usd::USDData usdData(filename);
    EXPECT_EQ(0u, usdData.Init().size());
    EXPECT_EQ(0u, usdData.ParseMaterials().size());

    EXPECT_EQ(1u, usdData.AllReferences().size());
    EXPECT_EQ(2u, usdData.Models().size());
    EXPECT_EQ(7u, usdData.Materials().size());

    auto materials = usdData.Materials();

    auto material0 = materials["Material_0"];
    EXPECT_EQ(ignition::math::Color(0.8, 0.8, 0.8), material0.Diffuse());
    EXPECT_EQ(ignition::math::Color(0, 0, 0), material0.Emissive());

    auto material1 = materials["Material_1"];
    EXPECT_EQ(ignition::math::Color(1, 0, 0), material1.Diffuse());
    EXPECT_EQ(ignition::math::Color(0, 0, 0), material1.Emissive());

    auto material2 = materials["Material_2"];
    EXPECT_EQ(ignition::math::Color(0, 1, 0), material2.Diffuse());
    EXPECT_EQ(ignition::math::Color(0, 0, 0), material2.Emissive());

    auto material3 = materials["Material_3"];
    EXPECT_EQ(ignition::math::Color(0, 0, 1), material3.Diffuse());
    EXPECT_EQ(ignition::math::Color(0, 0, 0), material3.Emissive());

    auto material4 = materials["Material_4"];
    EXPECT_EQ(ignition::math::Color(1, 1, 0), material4.Diffuse());
    EXPECT_EQ(ignition::math::Color(0, 0, 0), material4.Emissive());

    auto material5 = materials["Material_5"];
    EXPECT_EQ(ignition::math::Color(1, 0, 1), material5.Diffuse());
    EXPECT_EQ(ignition::math::Color(0, 0, 0), material5.Emissive());

    auto materialTextures = materials["Material_textures"];
    const auto * pbr = materialTextures.PbrMaterial();
    ASSERT_TRUE(pbr);
    const auto * workflow = pbr->Workflow(sdf::PbrWorkflowType::METAL);
    ASSERT_TRUE(workflow);
    EXPECT_EQ(ignition::math::Color(1, 1, 1), materialTextures.Diffuse());
    EXPECT_EQ(
      "materials/textures/FANS_Albedo.png",
      workflow->AlbedoMap());
    EXPECT_EQ(ignition::math::Color(0, 0, 0), materialTextures.Emissive());
    EXPECT_DOUBLE_EQ(0.5, workflow->Metalness());
    EXPECT_EQ(
      "materials/textures/FANS_Metalness.png",
      workflow->MetalnessMap());
    EXPECT_DOUBLE_EQ(0.5, workflow->Roughness());
    EXPECT_EQ(
      "materials/textures/FANS_Roughness.png",
      workflow->RoughnessMap());
    EXPECT_EQ(
      "materials/textures/FANS_Normal.png",
      workflow->NormalMap());

    // Find a path inside the stage
    auto boxStage = usdData.FindStage("box");
    EXPECT_EQ(filename, boxStage.first);
    EXPECT_EQ("Z", boxStage.second->UpAxis());
    EXPECT_DOUBLE_EQ(0.01, boxStage.second->MetersPerUnit());

    // Try to find a invalid path in the stage data
    auto invalidStage = usdData.FindStage("invalid");
    EXPECT_EQ("", invalidStage.first);
    EXPECT_EQ(nullptr, invalidStage.second);

    // Remove copied materials
    ignition::common::removeAll(
      ignition::common::joinPaths(ignition::common::cwd(), "materials"));
  }
}
