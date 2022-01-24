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
#include <sdf/usd/usd_parser/USDData.hh>

#include <ignition/utilities/ExtraTestMacros.hh>

#include "test_config.h"
#include "test_utils.hh"

/////////////////////////////////////////////////
TEST(USDData, Constructor)
{
  std::string pathBase = PROJECT_SOURCE_PATH;
  pathBase = ignition::common::joinPaths(pathBase, "test", "usd");

  // Open a invalid USD file
  EXPECT_THROW(sdf::usd::USDData(ignition::common::joinPaths(pathBase,
    "/invalid_name")), std::invalid_argument);

  // Open a valid USD file
  {
    std::string filename = ignition::common::joinPaths(pathBase,
      "/upAxisZ.usda");
    sdf::usd::USDData usdData(filename);
    EXPECT_TRUE(usdData.Init());
    EXPECT_EQ(6, usdData.ParseMaterials());
    EXPECT_EQ(1u, usdData.GetAllReferences().size());
    EXPECT_EQ(2u, usdData.GetModels().size());

    // Find a path inside the stage
    auto boxStage = usdData.findStage("box");
    EXPECT_EQ(filename, boxStage.first);
    EXPECT_EQ("Z", boxStage.second->GetUpAxis());
    EXPECT_DOUBLE_EQ(0.01, boxStage.second->GetMetersPerUnit());

    // Try to find a invalid path in the stage data
    auto invalidStage = usdData.findStage("invalid");
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
