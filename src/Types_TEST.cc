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

#include <string>
#include <vector>

#include "sdf/Types.hh"

/////////////////////////////////////////////////
TEST(Types, split_nothing)
{
  std::vector<std::string> split = sdf::split("hello", "/");
  ASSERT_EQ(split.size(), 1UL);
  EXPECT_EQ(split[0], "hello");
}

/////////////////////////////////////////////////
TEST(Types, split_single)
{
  std::vector<std::string> split = sdf::split("hello/there", "/");
  ASSERT_EQ(split.size(), 2UL);
  EXPECT_EQ(split[0], "hello");
  EXPECT_EQ(split[1], "there");
}

/////////////////////////////////////////////////
TEST(Types, split_blank)
{
  std::vector<std::string> split = sdf::split("", "/");
  ASSERT_EQ(split.size(), 1UL);
  EXPECT_EQ(split[0], "");
}

/////////////////////////////////////////////////
TEST(Types, split_empty_splitter)
{
  std::vector<std::string> split = sdf::split("hello", "");
  ASSERT_EQ(split.size(), 1UL);
  EXPECT_EQ(split[0], "hello");
}

/////////////////////////////////////////////////
TEST(Types, split_empty_string_and_splitter)
{
  std::vector<std::string> split = sdf::split("", "");
  ASSERT_EQ(split.size(), 1UL);
  EXPECT_EQ(split[0], "");
}

/////////////////////////////////////////////////
TEST(Types, split_no_matches)
{
  std::vector<std::string> split = sdf::split("hello/there", ":");
  ASSERT_EQ(split.size(), 1UL);
  EXPECT_EQ(split[0], "hello/there");
}

/////////////////////////////////////////////////
TEST(Types, trim_nothing)
{
  std::string out = sdf::trim("hello");
  EXPECT_EQ(out, "hello");

  out = sdf::trim("   hello   ");
  EXPECT_EQ(out, "hello");

  out = sdf::trim("");
  EXPECT_EQ(out, "");

  out = sdf::trim(" hello there  ");
  EXPECT_EQ(out, "hello there");

  out = sdf::trim("     ");
  EXPECT_EQ(out, "");

  out = sdf::trim("\t\t");
  EXPECT_EQ(out, "");

  out = sdf::trim("\txyz\t");
  EXPECT_EQ(out, "xyz");
}

/////////////////////////////////////////////////
/// Main
int main(int argc, char **argv)
{
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
