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

#include <string>
#include <gtest/gtest.h>

#include "sdf/Root.hh"
#include "sdf/World.hh"
#include "sdf/Filesystem.hh"
#include "test_config.h"

/////////////////////////////////////////////////
TEST(DOMRoot, InvalidSDF)
{
  const std::string testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "sdf",
        "empty_invalid.sdf");

  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
  EXPECT_FALSE(errors.empty());
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::FILE_READ);
}

/////////////////////////////////////////////////
TEST(DOMRoot, NoVersion)
{
  const std::string testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "sdf",
        "empty_noversion.sdf");

  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
  EXPECT_FALSE(errors.empty());
  std::cout << errors[0].Message() << std::endl;
  EXPECT_EQ(errors[0].Code(), sdf::ErrorCode::FILE_READ);
}

/////////////////////////////////////////////////
TEST(DOMRoot, Load)
{
  const std::string testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "sdf",
        "empty.sdf");

  sdf::Root root;
  EXPECT_EQ(root.WorldCount(), 0u);
  EXPECT_TRUE(root.Load(testFile).empty());
  EXPECT_EQ(root.Version(), "1.6");
  EXPECT_EQ(root.WorldCount(), 1u);
  EXPECT_TRUE(root.WorldByIndex(0) != nullptr);
  EXPECT_TRUE(root.WorldByIndex(1) == nullptr);

  EXPECT_EQ(root.WorldByIndex(0)->Name(), "default");
}
