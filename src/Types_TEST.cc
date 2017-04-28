/*
 * Copyright 2017 Open Source Robotics Foundation
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

#include "sdf/Types.hh"

TEST(Types, split)
{
  std::vector<std::string> split = sdf::split("hello", "/");
  ASSERT_EQ(split.size(), 1UL);
  ASSERT_EQ(split[0], "hello");

  split = sdf::split("hello/there", "/");
  ASSERT_EQ(split.size(), 2UL);
  ASSERT_EQ(split[0], "hello");
  ASSERT_EQ(split[1], "there");

  split = sdf::split("", "/");
  ASSERT_EQ(split.size(), 1UL);
  ASSERT_EQ(split[0], "");

  split = sdf::split("hello", "");
  ASSERT_EQ(split.size(), 1UL);
  ASSERT_EQ(split[0], "hello");

  split = sdf::split("", "");
  ASSERT_EQ(split.size(), 1UL);
  ASSERT_EQ(split[0], "");

  split = sdf::split("hello/there", ":");
  ASSERT_EQ(split.size(), 1UL);
  ASSERT_EQ(split[0], "hello/there");
}

TEST(Types, trim)
{
  std::string out = sdf::trim("hello");
  ASSERT_EQ(out, "hello");

  out = sdf::trim("   hello   ");
  ASSERT_EQ(out, "hello");

  out = sdf::trim("");
  ASSERT_EQ(out, "");

  out = sdf::trim(" hello there  ");
  ASSERT_EQ(out, "hello there");

  out = sdf::trim("     ");
  ASSERT_EQ(out, "");
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
