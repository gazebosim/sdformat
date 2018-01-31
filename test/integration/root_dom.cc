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
#include "sdf/Model.hh"
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

  EXPECT_EQ(root.WorldByIndex(0)->ModelCount(), 1u);
  ASSERT_TRUE(root.WorldByIndex(0)->ModelByIndex(0) != nullptr );
  EXPECT_EQ(root.WorldByIndex(0)->ModelByIndex(0)->Name(), "ground_plane");
  EXPECT_TRUE(root.WorldByIndex(0)->ModelNameExists("ground_plane"));
}

/////////////////////////////////////////////////
TEST(DOMRoot, LoadMultipleModels)
{
  const std::string testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "sdf",
        "root_multiple_models.sdf");

  sdf::Root root;
  EXPECT_TRUE(root.Load(testFile).empty());
  EXPECT_EQ(root.ModelCount(), 3u);

  EXPECT_EQ(root.ModelByIndex(0)->Name(), "robot1");
  EXPECT_EQ(root.ModelByIndex(1)->Name(), "robot2");
  EXPECT_EQ(root.ModelByIndex(2)->Name(), "last_robot");

  EXPECT_FALSE(root.ModelNameExists("robot"));
  EXPECT_TRUE(root.ModelNameExists("robot1"));
  EXPECT_TRUE(root.ModelNameExists("robot2"));
  EXPECT_TRUE(root.ModelNameExists("last_robot"));
}

/////////////////////////////////////////////////
TEST(DOMRoot, LoadDuplicateModels)
{
  const std::string testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "sdf",
        "root_duplicate_models.sdf");

  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
  EXPECT_FALSE(errors.empty());
  EXPECT_EQ(root.ModelCount(), 1u);
}

/////////////////////////////////////////////////
TEST(DOMRoot, LoadMultipleModelsOneBade)
{
  const std::string testFile =
    sdf::filesystem::append(PROJECT_SOURCE_PATH, "test", "sdf",
        "root_multiple_models_one_bad.sdf");

  sdf::Root root;
  sdf::Errors errors = root.Load(testFile);
  EXPECT_FALSE(errors.empty());

  for (const auto &e : errors)
    std::cout << e.Message() << std::endl;


  EXPECT_EQ(root.ModelCount(), 2u);
  EXPECT_TRUE(root.ModelNameExists("robot1"));
  EXPECT_TRUE(root.ModelNameExists("robot2"));

}
