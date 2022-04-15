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

#include <ignition/utils/ExtraTestMacros.hh>

#include <sdf/usd/usd_parser/USDStage.hh>
#include <sdf/usd/UsdError.hh>

#include "test_config.h"
#include "test_utils.hh"

/////////////////////////////////////////////////
TEST(USDStage, Constructor)
{
  // Check up Axis equal to Z and metersPerUnit
  {
    std::string filename = sdf::testing::TestFile("usd", "upAxisZ.usda");
    sdf::usd::USDStage stage(filename);
    sdf::usd::UsdErrors errors = stage.Init();
    EXPECT_EQ(0u, errors.size());

    EXPECT_EQ("Z", stage.UpAxis());
    EXPECT_DOUBLE_EQ(0.01, stage.MetersPerUnit());
    EXPECT_EQ(26u, stage.USDPaths().size());
  }

  // Check up Axis equal to Y and metersPerUnit
  {
    std::string filename = sdf::testing::TestFile("usd", "upAxisY.usda");
    sdf::usd::USDStage stage(filename);
    sdf::usd::UsdErrors errors = stage.Init();
    EXPECT_EQ(0u, errors.size());

    EXPECT_EQ("Y", stage.UpAxis());
    EXPECT_DOUBLE_EQ(1.0, stage.MetersPerUnit());
    EXPECT_EQ(9u, stage.USDPaths().size());
  }

  // Wrong upaxis
  {
    sdf::usd::USDStage stage(
      sdf::testing::TestFile("usd", "/upAxis_wrong.usda"));
    sdf::usd::UsdErrors errors = stage.Init();
    EXPECT_EQ(1u, errors.size());
  }

  // Invalid file
  {
    sdf::usd::USDStage stage(sdf::testing::TestFile("usd", "invalid_name"));
    sdf::usd::UsdErrors errors = stage.Init();
    EXPECT_EQ(1u, errors.size());
  }
}
