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

#include <sdf/usd/usd_parser/USDStage.hh>

#include "test_config.h"
#include "test_utils.hh"

/////////////////////////////////////////////////
TEST(USDStage, Constructor)
{
  std::string pathBase = PROJECT_SOURCE_PATH;
  pathBase = ignition::common::joinPaths(pathBase, "test", "usd");

  // Check up Axis equal to Z and metersPerUnit
  {
    std::string filename = ignition::common::joinPaths(pathBase,
      "/upAxisZ.usda");
    sdf::usd::USDStage stage(filename);
    sdf::Errors errors = stage.Init();
    EXPECT_EQ(0u, errors.size());

    EXPECT_EQ("Z", stage.GetUpAxis());
    EXPECT_DOUBLE_EQ(0.01, stage.GetMetersPerUnit());
    EXPECT_EQ(23u, stage.GetUSDPaths().size());
  }

  // Check up Axis equal to Y and metersPerUnit
  {
    std::string filename = ignition::common::joinPaths(pathBase,
      "/upAxisY.usda");
    sdf::usd::USDStage stage(filename);
    sdf::Errors errors = stage.Init();
    EXPECT_EQ(0u, errors.size());

    EXPECT_EQ("Y", stage.GetUpAxis());
    EXPECT_DOUBLE_EQ(1.0, stage.GetMetersPerUnit());
    EXPECT_EQ(10u, stage.GetUSDPaths().size());
  }

  // Wrong upaxis
  {
    sdf::usd::USDStage stage(ignition::common::joinPaths(pathBase,
      "/upAxis_wrong.usda"));
    sdf::Errors errors = stage.Init();
    EXPECT_EQ(1u, errors.size());
  }

  // Invalid file
  {
    sdf::usd::USDStage stage(ignition::common::joinPaths(pathBase,
      "/invalid_name"));
    sdf::Errors errors = stage.Init();
    EXPECT_EQ(1u, errors.size());
  }
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
