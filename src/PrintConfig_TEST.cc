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

#include "sdf/Assert.hh"
#include "sdf/PrintConfig.hh"

/////////////////////////////////////////////////
TEST(PrintConfig, Construction)
{
  sdf::PrintConfig config;
  EXPECT_FALSE(config.GetRotationInDegrees());
  EXPECT_FALSE(config.GetRotationSnapToDegrees());
}

/////////////////////////////////////////////////
TEST(PrintConfig, RotationInDegrees)
{
  sdf::PrintConfig config;
  EXPECT_FALSE(config.GetRotationInDegrees());

  config.SetRotationInDegrees(true);
  EXPECT_TRUE(config.GetRotationInDegrees());

  config.SetRotationInDegrees(false);
  EXPECT_FALSE(config.GetRotationInDegrees());
}

/////////////////////////////////////////////////
TEST(PrintConfig, RotationSnapToDegrees)
{
  sdf::PrintConfig config;
  EXPECT_FALSE(config.GetRotationSnapToDegrees().has_value());
  EXPECT_FALSE(config.GetRotationSnapTolerance().has_value());

  EXPECT_TRUE(config.SetRotationSnapToDegrees(5, 0.01));
  ASSERT_TRUE(config.GetRotationSnapToDegrees().has_value());
  EXPECT_EQ(5u, config.GetRotationSnapToDegrees().value());
  ASSERT_TRUE(config.GetRotationSnapTolerance().has_value());
  EXPECT_DOUBLE_EQ(0.01, config.GetRotationSnapTolerance().value());

  EXPECT_FALSE(config.SetRotationSnapToDegrees(0, 0.01));
  EXPECT_FALSE(config.SetRotationSnapToDegrees(360 + 1, 0.01));
  EXPECT_FALSE(config.SetRotationSnapToDegrees(5, -1e6));
  EXPECT_FALSE(config.SetRotationSnapToDegrees(5, 360 + 1e-6));

  EXPECT_FALSE(config.SetRotationSnapToDegrees(5, 5 + 1e-6));
  EXPECT_TRUE(config.SetRotationSnapToDegrees(5, 5 - 1e-6));
  ASSERT_TRUE(config.GetRotationSnapToDegrees().has_value());
  EXPECT_EQ(5u, config.GetRotationSnapToDegrees().value());
  ASSERT_TRUE(config.GetRotationSnapTolerance().has_value());
  EXPECT_DOUBLE_EQ(5 - 1e-6, config.GetRotationSnapTolerance().value());
}

/////////////////////////////////////////////////
TEST(PrintConfig, Compare)
{
  sdf::PrintConfig first;
  sdf::PrintConfig second;
  EXPECT_TRUE(first == second);
  EXPECT_TRUE(second == first);

  first.SetRotationInDegrees(true);
  EXPECT_TRUE(first.GetRotationInDegrees());
  EXPECT_FALSE(second.GetRotationInDegrees());
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
