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

#include <ignition/utilities/ExtraTestMacros.hh>

#include <sdf/usd/usd_parser/USDData.hh>
#include <sdf/usd/UsdError.hh>

#include "test_config.h"
#include "test_utils.hh"

/////////////////////////////////////////////////
TEST(USDData, Constructor)
{
  // Open a invalid USD file
  sdf::usd::USDData data(sdf::testing::TestFile("usd", "invalid_name"));
  sdf::usd::UsdErrors errors = data.Init();
  EXPECT_EQ(1u, errors.size());

  // Open a valid USD file
  {
    std::string filename = sdf::testing::TestFile("usd", "upAxisZ.usda");
    sdf::usd::USDData usdData(filename);
    EXPECT_EQ(0u, usdData.Init().size());
    EXPECT_EQ(0u, usdData.ParseMaterials().size());
    EXPECT_EQ(1u, usdData.GetAllReferences().size());
    EXPECT_EQ(2u, usdData.GetModels().size());

    // Find a path inside the stage
    auto boxStage = usdData.FindStage("box");
    EXPECT_EQ(filename, boxStage.first);
    EXPECT_EQ("Z", boxStage.second->GetUpAxis());
    EXPECT_DOUBLE_EQ(0.01, boxStage.second->GetMetersPerUnit());

    // Try to find a invalid path in the stage data
    auto invalidStage = usdData.FindStage("invalid");
    EXPECT_EQ("", invalidStage.first);
    EXPECT_EQ(nullptr, invalidStage.second);
  }
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
