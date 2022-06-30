/*
 * Copyright 2021 Open Source Robotics Foundation
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

#include "sdf/PrintConfig.hh"
#include "test_config.hh"

/////////////////////////////////////////////////
TEST(PrintConfig, Construction)
{
  sdf::PrintConfig config;
  EXPECT_FALSE(config.RotationInDegrees());
  EXPECT_FALSE(config.RotationSnapToDegrees());
  EXPECT_FALSE(config.PreserveIncludes());
}

/////////////////////////////////////////////////
TEST(PrintConfig, RotationInDegrees)
{
  sdf::PrintConfig config;
  EXPECT_FALSE(config.RotationInDegrees());

  config.SetRotationInDegrees(true);
  EXPECT_TRUE(config.RotationInDegrees());

  config.SetRotationInDegrees(false);
  EXPECT_FALSE(config.RotationInDegrees());
}

/////////////////////////////////////////////////
TEST(PrintConfig, RotationSnapToDegrees)
{
  sdf::PrintConfig config;
  EXPECT_FALSE(config.RotationSnapToDegrees().has_value());
  EXPECT_FALSE(config.RotationSnapTolerance().has_value());

  EXPECT_TRUE(config.SetRotationSnapToDegrees(5, 0.01));
  ASSERT_TRUE(config.RotationSnapToDegrees().has_value());
  EXPECT_EQ(5u, config.RotationSnapToDegrees().value());
  ASSERT_TRUE(config.RotationSnapTolerance().has_value());
  EXPECT_DOUBLE_EQ(0.01, config.RotationSnapTolerance().value());

  EXPECT_FALSE(config.SetRotationSnapToDegrees(0, 0.01));
  EXPECT_FALSE(config.SetRotationSnapToDegrees(360 + 1, 0.01));
  EXPECT_FALSE(config.SetRotationSnapToDegrees(5, -1e6));
  EXPECT_FALSE(config.SetRotationSnapToDegrees(5, 360 + 1e-6));

  EXPECT_FALSE(config.SetRotationSnapToDegrees(5, 5 + 1e-6));
  EXPECT_TRUE(config.SetRotationSnapToDegrees(5, 5 - 1e-6));
  ASSERT_TRUE(config.RotationSnapToDegrees().has_value());
  EXPECT_EQ(5u, config.RotationSnapToDegrees().value());
  ASSERT_TRUE(config.RotationSnapTolerance().has_value());
  EXPECT_DOUBLE_EQ(5 - 1e-6, config.RotationSnapTolerance().value());
}

/////////////////////////////////////////////////
TEST(PrintConfig, Compare)
{
  sdf::PrintConfig first;
  sdf::PrintConfig second;
  EXPECT_TRUE(first == second);
  EXPECT_TRUE(second == first);

  first.SetRotationInDegrees(true);
  EXPECT_TRUE(first.RotationInDegrees());
  EXPECT_FALSE(second.RotationInDegrees());
  EXPECT_FALSE(first == second);
  EXPECT_FALSE(second == first);

  second.SetRotationInDegrees(true);
  EXPECT_TRUE(first == second);
  EXPECT_TRUE(second == first);

  EXPECT_TRUE(first.SetRotationSnapToDegrees(5, 0.01));
  EXPECT_FALSE(first == second);
  EXPECT_FALSE(second == first);

  EXPECT_TRUE(second.SetRotationSnapToDegrees(5, 0.01));
  EXPECT_TRUE(first == second);
  EXPECT_TRUE(second == first);
}

/////////////////////////////////////////////////
TEST(PrintConfig, PreserveIncludes)
{
  sdf::PrintConfig config;
  EXPECT_FALSE(config.PreserveIncludes());
  config.SetPreserveIncludes(true);
  EXPECT_TRUE(config.PreserveIncludes());
  config.SetPreserveIncludes(false);
  EXPECT_FALSE(config.PreserveIncludes());
}

/////////////////////////////////////////////////
TEST(PrintConfig, OutPrecision)
{
  sdf::PrintConfig config;
  EXPECT_EQ(std::numeric_limits<int>::max(), config.OutPrecision());
  config.SetOutPrecision(2);
  EXPECT_EQ(2, config.OutPrecision());
  config.SetOutPrecision(8);
  EXPECT_EQ(8, config.OutPrecision());
  config.SetOutPrecision(std::numeric_limits<int>::max());
  EXPECT_EQ(std::numeric_limits<int>::max(), config.OutPrecision());
}
